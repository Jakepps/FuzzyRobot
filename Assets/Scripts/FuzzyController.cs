using System;
using UnityEngine;

namespace FuzzyRobot
{
    public readonly struct FuzzyOutput
    {
        public readonly float DeltaV; // диапазон как в курсовой: [-30..+30] (fuzzy units)
        public readonly float Phi;    // диапазон как в курсовой: [-45..+45] градусов
        public FuzzyOutput(float deltaV, float phi)
        {
            DeltaV = deltaV; 
            Phi = phi;
        }
    }

    /// <summary>
    /// Mamdani Fuzzy Controller (trimf + min/max + centroid) портирован из Python-кода курсовой.
    /// Входы: left/center/right (0..10), angle_err (-90..90), speed (0..100).
    /// Опционально: batt (0..100), surface (0..2), illum (0..1000).
    /// </summary>
    public static class FuzzyMamdaniController
    {
        // ---- Universe sampling (как в курсовой: x_dv [-30..30], x_phi [-45..45]) ----
        private const float DvMin = -30f;
        private const float DvMax =  30f;
        private const float DvStep = 0.5f;

        private const float PhiMin = -45f;
        private const float PhiMax =  45f;
        private const float PhiStep = 1f;

        private static readonly float[] X_DV = BuildRange(DvMin, DvMax, DvStep);
        private static readonly float[] X_PHI = BuildRange(PhiMin, PhiMax, PhiStep);

        // ---- Output membership functions (как в курсовой) ----
        private static readonly float[] MF_DV_STRONG_DEC = BuildTrimf(X_DV, -30, -30, -10);
        private static readonly float[] MF_DV_SLIGHT_DEC = BuildTrimf(X_DV, -20, -10,   0);
        private static readonly float[] MF_DV_NO_CHANGE  = BuildTrimf(X_DV,  -5,   0,   5);
        private static readonly float[] MF_DV_SLIGHT_INC = BuildTrimf(X_DV,   0,  10,  20);
        private static readonly float[] MF_DV_STRONG_INC = BuildTrimf(X_DV,  10,  30,  30);

        private static readonly float[] MF_PHI_SHARP_LEFT   = BuildTrimf(X_PHI, -45, -45, -15);
        private static readonly float[] MF_PHI_SLIGHT_LEFT  = BuildTrimf(X_PHI, -30, -15,   0);
        private static readonly float[] MF_PHI_STRAIGHT     = BuildTrimf(X_PHI, -10,   0,  10);
        private static readonly float[] MF_PHI_SLIGHT_RIGHT = BuildTrimf(X_PHI,   0,  15,  30);
        private static readonly float[] MF_PHI_SHARP_RIGHT  = BuildTrimf(X_PHI,  15,  45,  45);

        private enum DvTerm { StrongDec, SlightDec, NoChange, SlightInc, StrongInc }
        private enum PhiTerm { SharpLeft, SlightLeft, Straight, SlightRight, SharpRight }

        // Маппинг выходных термов к правилам (1..34) — один-в-один как в Python (dv_mfs/phi_mfs)
        private static readonly DvTerm[] DV_TERM_BY_RULE =
        {
            // r1..r34
            DvTerm.StrongDec, DvTerm.SlightDec, DvTerm.SlightDec, DvTerm.NoChange,  DvTerm.NoChange,
            DvTerm.StrongDec, DvTerm.SlightDec, DvTerm.SlightDec, DvTerm.NoChange,  DvTerm.NoChange,
            DvTerm.SlightInc, DvTerm.StrongInc, DvTerm.SlightDec, DvTerm.NoChange,  DvTerm.SlightInc,
            DvTerm.SlightDec, DvTerm.StrongInc, DvTerm.StrongDec, DvTerm.SlightDec, DvTerm.NoChange,
            DvTerm.StrongDec, DvTerm.StrongDec, DvTerm.SlightDec, DvTerm.NoChange,  DvTerm.StrongDec,
            DvTerm.SlightDec, DvTerm.SlightInc, DvTerm.StrongDec, DvTerm.StrongDec, DvTerm.SlightDec,
            DvTerm.StrongDec, DvTerm.StrongDec, DvTerm.NoChange,  DvTerm.SlightInc
        };

        private static readonly PhiTerm[] PHI_TERM_BY_RULE =
        {
            // r1..r34
            PhiTerm.Straight,   PhiTerm.SharpRight, PhiTerm.SharpLeft,  PhiTerm.SlightRight, PhiTerm.SlightLeft,
            PhiTerm.Straight,   PhiTerm.SharpRight, PhiTerm.SharpLeft,  PhiTerm.SlightRight, PhiTerm.SlightLeft,
            PhiTerm.Straight,   PhiTerm.Straight,   PhiTerm.Straight,   PhiTerm.Straight,    PhiTerm.Straight,
            PhiTerm.SlightLeft, PhiTerm.Straight,   PhiTerm.SharpRight, PhiTerm.Straight,    PhiTerm.SharpLeft,
            PhiTerm.Straight,   PhiTerm.SharpRight, PhiTerm.SlightRight,PhiTerm.Straight,    PhiTerm.Straight,
            PhiTerm.SlightRight,PhiTerm.Straight,   PhiTerm.Straight,   PhiTerm.Straight,    PhiTerm.Straight,
            PhiTerm.Straight,   PhiTerm.SharpLeft,  PhiTerm.Straight,   PhiTerm.SlightRight
        };

        /// <summary>
        /// Основной шаг контроллера.
        /// left/center/right: дистанции (0..10, где 10 = "далеко/ничего не видим")
        /// angleErrDeg: ошибка курса в градусах (-90..90)
        /// speed: скорость в fuzzy units (0..100)
        /// useExtremeFactors: если false — правила r21..r34 не участвуют (как вы просили сейчас)
        /// </summary>
        public static FuzzyOutput Compute(
            float left, float center, float right,
            float angleErrDeg, float speed,
            bool useExtremeFactors = false,
            float batt = 50f, float surface = 0f, float illum = 200f)
        {
            // ---- 1) Fuzzification (как в курсовой) ----
            float ld_close = TrimfScalar(left,   0,   0,  5);
            float ld_med   = TrimfScalar(left, 2.5f,  5, 7.5f);
            float ld_far   = TrimfScalar(left,   5,  10, 10);

            float cd_close = TrimfScalar(center, 0,   0,  5);
            float cd_med   = TrimfScalar(center, 2.5f, 5, 7.5f);
            float cd_far   = TrimfScalar(center, 5,  10, 10);

            float rd_close = TrimfScalar(right,  0,   0,  5);
            float rd_med   = TrimfScalar(right, 2.5f,  5, 7.5f);
            float rd_far   = TrimfScalar(right,  5,  10, 10);

            float a_bl  = TrimfScalar(angleErrDeg, -90, -90, -30);
            float a_sl  = TrimfScalar(angleErrDeg, -30, -10,   0);
            float a_tgt = TrimfScalar(angleErrDeg, -15,   0,  15);
            float a_sr  = TrimfScalar(angleErrDeg,   0,  10,  30);
            float a_br  = TrimfScalar(angleErrDeg,  30,  90,  90);

            float sp_lo = TrimfScalar(speed,   0,   0,  30);
            float sp_me = TrimfScalar(speed,  20,  50,  80);
            float sp_hi = TrimfScalar(speed,  70, 100, 100);

            // Экстремальные факторы (оставлены для будущего)
            float b_lo = TrimfScalar(batt,    0,   0,  20);
            float b_me = TrimfScalar(batt,   15,  60, 100);
            float b_hi = TrimfScalar(batt,   50, 100, 100);

            float su_sm = TrimfScalar(surface, 0,   0,   1);
            float su_ro = TrimfScalar(surface, 0.5f, 1, 1.5f);
            float su_sl = TrimfScalar(surface, 1,   2,   2);

            float i_no = TrimfScalar(illum,    0,    0,   50);
            float i_fo = TrimfScalar(illum,   50,  200, 1000);
            float i_br = TrimfScalar(illum,  200, 1000, 1000);

            // ---- 2) Rules (r1..r34) ----
            float[] r = new float[34];

            // Группа 1: Избегание препятствий
            r[0] = Min(cd_close, sp_hi);             // r1
            r[1] = Min(ld_close, rd_far);            // r2
            r[2] = Min(rd_close, ld_far);            // r3
            r[3] = Min(ld_med, cd_far);              // r4
            r[4] = Min(rd_med, cd_far);              // r5
            r[5] = Min(ld_close, rd_close);          // r6

            // Группа 2: Коррекция курса
            r[6]  = a_bl;                            // r7
            r[7]  = a_br;                            // r8
            r[8]  = a_sl;                            // r9
            r[9]  = a_sr;                            // r10
            r[10] = Min(a_tgt, sp_lo);               // r11

            // Группа 3: Управление скоростью
            r[11] = Min(sp_lo, cd_far);              // r12
            r[12] = Min(sp_hi, cd_med);              // r13
            r[13] = Min(sp_me, ld_far, cd_far, rd_far); // r14
            r[14] = Min(sp_lo, a_tgt);               // r15

            // Группа 4: Смешанные сценарии
            r[15] = Min(cd_med, a_sr);               // r16
            r[16] = Min(ld_far, rd_far, a_tgt);      // r17
            r[17] = Min(cd_close, a_bl);             // r18
            r[18] = Min(ld_med, rd_med, a_tgt);      // r19
            r[19] = Min(cd_far, a_br);               // r20

            if (useExtremeFactors)
            {
                // Группа 5: Уровень заряда батареи
                r[20] = Min(b_lo, cd_far);           // r21
                r[21] = Min(b_lo, cd_close);         // r22
                r[22] = Min(b_me, su_sl);            // r23
                r[23] = Min(b_hi, i_no);             // r24

                // Группа 6: Тип поверхности
                r[24] = Min(su_sl, sp_hi);           // r25
                r[25] = Min(su_ro, cd_close);        // r26
                r[26] = Min(su_sm, a_tgt);           // r27

                // Группа 7: Освещённость
                r[27] = Min(i_fo, cd_med);           // r28
                r[28] = Min(i_no, b_lo);             // r29
                r[29] = Min(i_br, su_sl);            // r30

                // Группа 8: Сложные условия
                r[30] = Min(b_lo, su_sl, i_fo);      // r31
                r[31] = cd_close;                    // r32
                r[32] = Min(su_ro, b_me);            // r33
                r[33] = Min(i_no, cd_far);           // r34
            }
            else
            {
                // r21..r34 выключаем
                for (int i = 20; i < 34; i++)
                {
                    r[i] = 0f;
                }
            }

            // ---- 3) Aggregation via term activations (эффективнее, чем 34 массива) ----
            float[] dvAct  = new float[5]; // по DvTerm
            float[] phiAct = new float[5]; // по PhiTerm

            for (int i = 0; i < 34; i++)
            {
                float ri = r[i];
                if (ri <= 0f) continue;

                dvAct[(int)DV_TERM_BY_RULE[i]] = Mathf.Max(dvAct[(int)DV_TERM_BY_RULE[i]], ri);
                phiAct[(int)PHI_TERM_BY_RULE[i]] = Mathf.Max(phiAct[(int)PHI_TERM_BY_RULE[i]], ri);
            }

            // ---- 4) Defuzzification (centroid) ----
            float dv  = DefuzzifyCentroidDV(dvAct);
            float phi = DefuzzifyCentroidPhi(phiAct);

            return new FuzzyOutput(dv, phi);
        }

        // ===== Defuzzification helpers =====

        private static float DefuzzifyCentroidDV(float[] act)
        {
            // mu_out(x) = max_t min(act[t], MF[t](x))
            float sumW = 0f, sumMu = 0f;

            for (int i = 0; i < X_DV.Length; i++)
            {
                float mu = 0f;
                mu = Mathf.Max(mu, Mathf.Min(act[(int)DvTerm.StrongDec], MF_DV_STRONG_DEC[i]));
                mu = Mathf.Max(mu, Mathf.Min(act[(int)DvTerm.SlightDec], MF_DV_SLIGHT_DEC[i]));
                mu = Mathf.Max(mu, Mathf.Min(act[(int)DvTerm.NoChange],  MF_DV_NO_CHANGE[i]));
                mu = Mathf.Max(mu, Mathf.Min(act[(int)DvTerm.SlightInc], MF_DV_SLIGHT_INC[i]));
                mu = Mathf.Max(mu, Mathf.Min(act[(int)DvTerm.StrongInc], MF_DV_STRONG_INC[i]));

                sumW  += X_DV[i] * mu;
                sumMu += mu;
            }

            return (sumMu <= 1e-6f) ? 0f : (sumW / sumMu);
        }

        private static float DefuzzifyCentroidPhi(float[] act)
        {
            float sumW = 0f, sumMu = 0f;

            for (int i = 0; i < X_PHI.Length; i++)
            {
                float mu = 0f;
                mu = Mathf.Max(mu, Mathf.Min(act[(int)PhiTerm.SharpLeft],   MF_PHI_SHARP_LEFT[i]));
                mu = Mathf.Max(mu, Mathf.Min(act[(int)PhiTerm.SlightLeft],  MF_PHI_SLIGHT_LEFT[i]));
                mu = Mathf.Max(mu, Mathf.Min(act[(int)PhiTerm.Straight],    MF_PHI_STRAIGHT[i]));
                mu = Mathf.Max(mu, Mathf.Min(act[(int)PhiTerm.SlightRight], MF_PHI_SLIGHT_RIGHT[i]));
                mu = Mathf.Max(mu, Mathf.Min(act[(int)PhiTerm.SharpRight],  MF_PHI_SHARP_RIGHT[i]));

                sumW  += X_PHI[i] * mu;
                sumMu += mu;
            }

            return (sumMu <= 1e-6f) ? 0f : (sumW / sumMu);
        }

        // ===== Math utilities =====

        private static float TrimfScalar(float x, float a, float b, float c)
        {
            // стандартный trimf
            if (x <= a || x >= c) return 0f;
            if (Math.Abs(b - a) < 1e-6f && Mathf.Approximately(x, a)) return 1f; // "плоская вершина" слева
            if (Math.Abs(c - b) < 1e-6f && Mathf.Approximately(x, c)) return 1f; // "плоская вершина" справа

            if (x < b) return (x - a) / (b - a);
            if (x > b) return (c - x) / (c - b);
            return 1f;
        }

        private static float[] BuildRange(float min, float max, float step)
        {
            int n = Mathf.FloorToInt((max - min) / step) + 1;
            var arr = new float[n];
            for (int i = 0; i < n; i++) arr[i] = min + i * step;
            return arr;
        }

        private static float[] BuildTrimf(float[] x, float a, float b, float c)
        {
            var y = new float[x.Length];
            for (int i = 0; i < x.Length; i++) y[i] = TrimfScalar(x[i], a, b, c);
            return y;
        }

        private static float Min(float a, float b) => Mathf.Min(a, b);
        private static float Min(float a, float b, float c) => Mathf.Min(a, Mathf.Min(b, c));
        private static float Min(float a, float b, float c, float d) => Mathf.Min(Mathf.Min(a, b), Mathf.Min(c, d));
    }
}
