using System;
using UnityEngine;

namespace FuzzyRobot
{
    public readonly struct FuzzyControlOutput
    {
        public readonly float DeltaV; // [-30..30]
        public readonly float Phi;    // [-45..45]

        public FuzzyControlOutput(float deltaV, float phi)
        {
            DeltaV = deltaV;
            Phi = phi;
        }
    }

    /// <summary>
    /// Чистое доменное ядро нечеткого контроллера.
    /// Не зависит от Unity API, кроме Mathf/Debug-friendly типов float.
    /// </summary>
    public sealed class FuzzyMamdaniController
    {
        private const float Eps = 1e-6f;

        private static readonly float[] XDv = Linspace(-30f, 30f, 100);
        private static readonly float[] XPhi = Linspace(-45f, 45f, 100);

        // ΔV
        private static readonly float[] MuDvStrongDec = BuildTrimf(XDv, -30f, -30f, -10f);
        private static readonly float[] MuDvSlightDec = BuildTrimf(XDv, -20f, -10f, 0f);
        private static readonly float[] MuDvNoChange  = BuildTrimf(XDv, -5f, 0f, 5f);
        private static readonly float[] MuDvSlightInc = BuildTrimf(XDv, 0f, 10f, 20f);
        private static readonly float[] MuDvStrongInc = BuildTrimf(XDv, 10f, 30f, 30f);

        // φ
        private static readonly float[] MuPhiSharpLeft   = BuildTrimf(XPhi, -45f, -45f, -15f);
        private static readonly float[] MuPhiSlightLeft  = BuildTrimf(XPhi, -30f, -15f, 0f);
        private static readonly float[] MuPhiStraight    = BuildTrimf(XPhi, -10f, 0f, 10f);
        private static readonly float[] MuPhiSlightRight = BuildTrimf(XPhi, 0f, 15f, 30f);
        private static readonly float[] MuPhiSharpRight  = BuildTrimf(XPhi, 15f, 45f, 45f);

        private static readonly float[][] DvRuleMfs =
        {
            MuDvStrongDec, MuDvSlightDec, MuDvSlightDec, MuDvNoChange,  MuDvNoChange,
            MuDvStrongDec, MuDvSlightDec, MuDvSlightDec, MuDvNoChange,  MuDvNoChange,
            MuDvSlightInc, MuDvStrongInc, MuDvSlightDec, MuDvNoChange,  MuDvSlightInc,
            MuDvSlightDec, MuDvStrongInc, MuDvStrongDec, MuDvSlightDec, MuDvNoChange,
            MuDvStrongDec, MuDvStrongDec, MuDvSlightDec, MuDvNoChange,  MuDvStrongDec,
            MuDvSlightDec, MuDvSlightInc, MuDvStrongDec, MuDvStrongDec, MuDvSlightDec,
            MuDvStrongDec, MuDvStrongDec, MuDvNoChange,  MuDvSlightInc
        };

        /// <summary>
        /// Для правил коррекции курса я подправил знаки поворота под фактическую семантику Unity SignedAngle:
        /// отрицательный угол = цель слева, положительный = цель справа.
        /// Иначе робот будет зеркалить курс.
        /// </summary>
        private static readonly float[][] PhiRuleMfs =
        {
            // 1..6 obstacle avoidance
            MuPhiStraight,    MuPhiSharpRight, MuPhiSharpLeft,  MuPhiSlightRight, MuPhiSlightLeft,
            MuPhiStraight,

            // 7..10 course correction (исправлено)
            MuPhiSharpLeft,   // r7  big left  -> sharp left
            MuPhiSharpRight,  // r8  big right -> sharp right
            MuPhiSlightLeft,  // r9  small left -> slight left
            MuPhiSlightRight, // r10 small right -> slight right

            // 11..15
            MuPhiStraight, MuPhiStraight, MuPhiStraight, MuPhiStraight, MuPhiStraight,

            // 16..20 mixed scenarios
            MuPhiSlightRight, // r16 center medium + small right
            MuPhiStraight,
            MuPhiSharpLeft,   // r18 center close + big left
            MuPhiStraight,
            MuPhiSharpRight,  // r20 center far + big right

            // 21..34 extreme / auxiliary
            MuPhiStraight,
            MuPhiSharpRight,
            MuPhiSlightRight,
            MuPhiStraight,
            MuPhiStraight,
            MuPhiSlightRight,
            MuPhiStraight,
            MuPhiStraight,
            MuPhiStraight,
            MuPhiStraight,
            MuPhiStraight,
            MuPhiSharpLeft,
            MuPhiStraight,
            MuPhiSlightRight
        };

        private readonly float[] _rules = new float[34];
        private readonly float[] _muDvOut = new float[100];
        private readonly float[] _muPhiOut = new float[100];

        public FuzzyControlOutput Compute(
            float left,
            float center,
            float right,
            float angleErrDeg,
            float speed,
            float battery = 50f,
            float surface = 0f,
            float illumination = 200f)
        {
            Array.Clear(_muDvOut, 0, _muDvOut.Length);
            Array.Clear(_muPhiOut, 0, _muPhiOut.Length);

            // --------------------------------
            // 1) Фаззификация
            // --------------------------------

            float ldClose = Trimf(left,   0f, 0f, 5f);
            float ldMed   = Trimf(left,   2.5f, 5f, 7.5f);
            float ldFar   = Trimf(left,   5f, 10f, 10f);

            float cdClose = Trimf(center, 0f, 0f, 5f);
            float cdMed   = Trimf(center, 2.5f, 5f, 7.5f);
            float cdFar   = Trimf(center, 5f, 10f, 10f);

            float rdClose = Trimf(right,  0f, 0f, 5f);
            float rdMed   = Trimf(right,  2.5f, 5f, 7.5f);
            float rdFar   = Trimf(right,  5f, 10f, 10f);

            float aBl  = Trimf(angleErrDeg, -90f, -90f, -30f);
            float aSl  = Trimf(angleErrDeg, -30f, -10f,   0f);
            float aTgt = Trimf(angleErrDeg, -15f,   0f,  15f);
            float aSr  = Trimf(angleErrDeg,   0f,  10f,  30f);
            float aBr  = Trimf(angleErrDeg,  30f,  90f,  90f);

            float spLo = Trimf(speed, 0f,  0f,  30f);
            float spMe = Trimf(speed, 20f, 50f, 80f);
            float spHi = Trimf(speed, 70f, 100f, 100f);

            float bLo = Trimf(battery, 0f,  0f,  20f);
            float bMe = Trimf(battery, 15f, 60f, 100f);
            float bHi = Trimf(battery, 50f, 100f, 100f);

            float suSm = Trimf(surface, 0f, 0f, 1f);
            float suRo = Trimf(surface, 0.5f, 1f, 1.5f);
            float suSl = Trimf(surface, 1f, 2f, 2f);

            float iNo = Trimf(illumination, 0f,   0f,   50f);
            float iFo = Trimf(illumination, 50f, 200f, 1000f);
            float iBr = Trimf(illumination, 200f, 1000f, 1000f);

            // --------------------------------
            // 2) Оценка правил
            // --------------------------------

            // Группа 1: Избегание препятствий
            _rules[0] = Min(cdClose, spHi);
            _rules[1] = Min(ldClose, rdFar);
            _rules[2] = Min(rdClose, ldFar);
            _rules[3] = Min(ldMed, cdFar);
            _rules[4] = Min(rdMed, cdFar);
            _rules[5] = Min(ldClose, rdClose);

            // Группа 2: Коррекция курса
            _rules[6]  = aBl;
            _rules[7]  = aBr;
            _rules[8]  = aSl;
            _rules[9]  = aSr;
            _rules[10] = Min(aTgt, spLo);

            // Группа 3: Управление скоростью
            _rules[11] = Min(spLo, cdFar);
            _rules[12] = Min(spHi, cdMed);
            _rules[13] = Min(spMe, ldFar, cdFar, rdFar);
            _rules[14] = Min(spLo, aTgt);

            // Группа 4: Смешанные сценарии
            _rules[15] = Min(cdMed, aSr);
            _rules[16] = Min(ldFar, rdFar, aTgt);
            _rules[17] = Min(cdClose, aBl);
            _rules[18] = Min(ldMed, rdMed, aTgt);
            _rules[19] = Min(cdFar, aBr);

            // Группа 5: Батарея
            _rules[20] = Min(bLo, cdFar);
            _rules[21] = Min(bLo, cdClose);
            _rules[22] = Min(bMe, suSl);
            _rules[23] = Min(bHi, iNo);

            // Группа 6: Поверхность
            _rules[24] = Min(suSl, spHi);
            _rules[25] = Min(suRo, cdClose);
            _rules[26] = Min(suSm, aTgt);

            // Группа 7: Освещённость
            _rules[27] = Min(iFo, cdMed);
            _rules[28] = Min(iNo, bLo);
            _rules[29] = Min(iBr, suSl);

            // Группа 8: Сложные условия
            _rules[30] = Min(bLo, suSl, iFo);
            _rules[31] = cdClose;
            _rules[32] = Min(suRo, bMe);
            _rules[33] = Min(iNo, cdFar);

            // --------------------------------
            // 3) Агрегация
            // --------------------------------
            Aggregate(_rules, DvRuleMfs, _muDvOut);
            Aggregate(_rules, PhiRuleMfs, _muPhiOut);

            // --------------------------------
            // 4) Дефаззификация
            // --------------------------------
            float dv = Defuzzify(XDv, _muDvOut);
            float phi = Defuzzify(XPhi, _muPhiOut);

            return new FuzzyControlOutput(dv, phi);
        }

        private static void Aggregate(float[] rules, float[][] ruleMfs, float[] output)
        {
            for (int i = 0; i < rules.Length; i++)
            {
                float r = rules[i];
                if (r <= 0f)
                {
                    continue;
                }

                float[] mf = ruleMfs[i];
                for (int j = 0; j < output.Length; j++)
                {
                    float clipped = Mathf.Min(r, mf[j]);
                    if (clipped > output[j])
                    {
                        output[j] = clipped;
                    }
                }
            }
        }

        private static float Defuzzify(float[] x, float[] mu)
        {
            float numerator = 0f;
            float denominator = 0f;

            for (int i = 0; i < x.Length; i++)
            {
                numerator += x[i] * mu[i];
                denominator += mu[i];
            }

            return numerator / (denominator + Eps);
        }

        private static float Trimf(float x, float a, float b, float c)
        {
            float left = (x - a) / ((b - a) + Eps);
            float right = (c - x) / ((c - b) + Eps);
            return Mathf.Max(Mathf.Min(left, right), 0f);
        }

        private static float[] BuildTrimf(float[] universe, float a, float b, float c)
        {
            var result = new float[universe.Length];
            for (int i = 0; i < universe.Length; i++)
            {
                result[i] = Trimf(universe[i], a, b, c);
            }
            
            return result;
        }

        private static float[] Linspace(float start, float end, int count)
        {
            var arr = new float[count];
            if (count == 1)
            {
                arr[0] = start;
                return arr;
            }

            float step = (end - start) / (count - 1);
            for (int i = 0; i < count; i++)
            {
                arr[i] = start + step * i;
            }

            return arr;
        }

        private static float Min(params float[] values)
        {
            float min = values[0];
            for (int i = 1; i < values.Length; i++)
            {
                if (values[i] < min)
                {
                    min = values[i];
                }
            }

            return min;
        }
    }
}
