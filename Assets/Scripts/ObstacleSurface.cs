using UnityEngine;

namespace FuzzyRobot
{
    public enum SurfaceKind
    {
        Default = 0,
        Wall = 1,
        // позже можно расширять: Road, Grass, Ice и т.п.
    }

    public sealed class ObstacleSurface : MonoBehaviour
    {
        public SurfaceKind Kind = SurfaceKind.Default;
    }
}
