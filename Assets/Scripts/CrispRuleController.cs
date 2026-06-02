using UnityEngine;

namespace FuzzyRobot
{
    public readonly struct CrispControlOutput
    {
        public readonly float DeltaV; // [-30..30]
        public readonly float Phi;    // [-45..45]

        public CrispControlOutput(float deltaV, float phi)
        {
            DeltaV = deltaV;
            Phi = phi;
        }
    }

    /// <summary>
    /// Четкий (crisp) контроллер движения робота.
    /// Классические правила «если–то» с жёсткими порогами: без функций принадлежности,
    /// без агрегации, без дефаззификации. Выход дискретный (ступенчатый/bang-bang).
    ///
    /// Контроллер намеренно повторяет интерфейс <see cref="FuzzyMamdaniController.Compute"/>
    /// (те же входы и те же диапазоны выходов ΔV∈[-30..30], φ∈[-45..45]),
    /// поэтому оба алгоритма взаимозаменяемы и сравниваются при одинаковой физике робота.
    /// </summary>
    public class CrispRuleController
    {
        // --- Пороги классификации дистанции (universe сенсоров 0..10 м) ---
        // CLOSE  : d < CloseDistance
        // MEDIUM : CloseDistance <= d < FarDistance
        // FAR    : d >= FarDistance
        public float CloseDistance = 3.5f;
        public float FarDistance = 6.5f;

        // --- Пороги классификации угла на цель (град) ---
        // Знак как в Unity SignedAngle: отрицательный = цель слева, положительный = справа.
        public float AlignedAngle = 10f; // |a| <= AlignedAngle  -> прямо
        public float SharpAngle = 30f;   // |a| >= SharpAngle    -> резкий поворот

        // --- Пороги классификации скорости (fuzzy-шкала 0..100) ---
        public float SpeedLow = 30f;
        public float SpeedHigh = 70f;

        // --- Дискретные уровни управления курсом (φ) ---
        private const float PhiSharp = 40f;
        private const float PhiSlight = 15f;
        private const float PhiStraight = 0f;

        // --- Дискретные уровни изменения скорости (ΔV) ---
        private const float DvStrongDec = -30f;
        private const float DvSlightDec = -10f;
        private const float DvNoChange = 0f;
        private const float DvSlightInc = 15f;
        private const float DvStrongInc = 30f;

        public CrispControlOutput Compute(
            float left,
            float center,
            float right,
            float angleErrDeg,
            float speed,
            float battery = 50f,
            float surface = 0f,
            float illumination = 200f)
        {
            // --------------------------------
            // 1) Жёсткая классификация входов
            // --------------------------------
            bool cClose = center < CloseDistance;
            bool cFar = center >= FarDistance;
            bool lClose = left < CloseDistance;
            bool rClose = right < CloseDistance;
            bool lFar = left >= FarDistance;
            bool rFar = right >= FarDistance;

            bool speedHigh = speed >= SpeedHigh;
            bool speedLow = speed <= SpeedLow;

            bool aligned = Mathf.Abs(angleErrDeg) <= AlignedAngle;

            // --------------------------------
            // 2) Управление курсом φ
            //    Приоритет: объезд препятствий, затем коррекция курса на цель.
            // --------------------------------
            float phi;

            if (cClose)
            {
                // Спереди препятствие — уходим в сторону с бо́льшим просветом.
                phi = left >= right ? -PhiSharp : PhiSharp;
            }
            else if (lClose && !rClose)
            {
                // Мешает слева — поворот вправо (резко, если справа совсем чисто).
                phi = rFar ? PhiSharp : PhiSlight;
            }
            else if (rClose && !lClose)
            {
                // Мешает справа — поворот влево.
                phi = lFar ? -PhiSharp : -PhiSlight;
            }
            else
            {
                // Путь свободен — ступенчатая коррекция курса на цель.
                if (angleErrDeg <= -SharpAngle)
                {
                    phi = -PhiSharp;
                }
                else if (angleErrDeg <= -AlignedAngle)
                {
                    phi = -PhiSlight;
                }
                else if (angleErrDeg < AlignedAngle)
                {
                    phi = PhiStraight;
                }
                else if (angleErrDeg < SharpAngle)
                {
                    phi = PhiSlight;
                }
                else
                {
                    phi = PhiSharp;
                }
            }

            // --------------------------------
            // 3) Управление скоростью ΔV
            // --------------------------------
            float dv;

            if (cClose)
            {
                dv = DvStrongDec;                 // тормозим перед препятствием
            }
            else if (lClose || rClose)
            {
                dv = DvSlightDec;                 // осторожно у боковой преграды
            }
            else if (!cFar && speedHigh)
            {
                dv = DvSlightDec;                 // среднее расстояние на высокой скорости
            }
            else if (cFar && aligned && speedLow)
            {
                dv = DvStrongInc;                 // чисто и по курсу — разгон
            }
            else if (cFar && !speedHigh)
            {
                dv = DvSlightInc;                 // чисто — поддерживаем разгон
            }
            else
            {
                dv = DvNoChange;                  // держим скорость
            }

            // --------------------------------
            // 4) Экстремальные факторы (те же входы, что и у нечеткого контроллера):
            //    низкий заряд, скользкая/шероховатая поверхность, темнота.
            // --------------------------------
            if (battery < 20f)
            {
                dv -= 10f;
            }

            if (surface >= 1.5f)
            {
                dv -= 10f;        // скользко
            }
            else if (surface >= 0.5f)
            {
                dv -= 5f;         // шероховато
            }

            if (illumination < 50f)
            {
                dv -= 10f;        // темно
            }

            dv = Mathf.Clamp(dv, -30f, 30f);
            phi = Mathf.Clamp(phi, -45f, 45f);

            return new CrispControlOutput(dv, phi);
        }
    }
}
