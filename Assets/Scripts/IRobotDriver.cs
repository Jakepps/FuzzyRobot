using UnityEngine;

namespace FuzzyRobot
{
    /// <summary>
    /// Общий контракт телеметрии для драйверов робота (нечеткого и четкого).
    /// Нужен, чтобы HUD и другие наблюдатели не зависели от конкретной реализации
    /// и одинаково работали в сценах сравнения алгоритмов.
    /// </summary>
    public interface IRobotDriver
    {
        bool HasReachedTarget { get; }
        Transform Target { get; }
        float MaxSpeedMs { get; }
    }
}
