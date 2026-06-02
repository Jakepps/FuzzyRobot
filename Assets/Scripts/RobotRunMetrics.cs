using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace FuzzyRobot
{
    /// <summary>
    /// Логгер метрик прохождения сцены для сравнения алгоритмов движения (нечеткий vs четкий).
    /// Считает показатели из раздела 1.3 диплома:
    ///   - достижение цели;
    ///   - время прохождения T;
    ///   - длина траектории L_факт;
    ///   - средняя скорость v_ср = L_факт / T            (формула 2);
    ///   - число столкновений;
    ///   - коэффициент эффективности η = L_эт / L_факт · 100%   (формула 3).
    ///
    /// По итогу прогона формирует/обновляет общий Markdown-отчёт со сравнительной таблицей
    /// по каждой сцене. Имя сцены нормализуется (суффиксы _Crisp/_Fuzzy отбрасываются),
    /// поэтому нечеткий и четкий прогоны одной сцены попадают в одну таблицу.
    ///
    /// Компонент сам цепляется к роботу при старте (как HUD), отдельная настройка не нужна.
    /// Для лабиринта добавьте компонент на робота вручную и задайте центральную линию прохода
    /// (referenceMode = Waypoints + referenceWaypoints) — тогда L_эт считается по ней.
    /// </summary>
    public class RobotRunMetrics : MonoBehaviour
    {
        public enum ReferenceMode
        {
            StraightLineStartToGoal, // L_эт = прямое расстояние старт→цель (свободный путь)
            Waypoints,               // L_эт = длина ломаной старт→точки→цель (лабиринт)
            Manual                   // L_эт задаётся вручную числом
        }

        [Header("Что измеряем")]
        [Tooltip("Слои, столкновение с которыми считается столкновением с препятствием (как у сенсоров — слой 3).")]
        [SerializeField] private LayerMask obstacleLayers = 1 << 3;
        [Tooltip("Начинать отсчёт времени и длины с момента первого движения (а не с момента старта сцены).")]
        [SerializeField] private bool startOnFirstMovement = true;
        [SerializeField] private float movementThreshold = 0.02f; // м/с — порог «начала движения»
        [SerializeField] private float pathDeadband = 0.001f;     // м — антидребезг накопления длины
        [SerializeField] private float collisionDebounce = 0.4f;  // с — чтобы один контакт не считался многократно

        [Header("Эталонная длина L_эт (для η)")]
        [SerializeField] private ReferenceMode referenceMode = ReferenceMode.StraightLineStartToGoal;
        [Tooltip("Точки центральной линии прохода (для лабиринта). L_эт = старт → точки → цель.")]
        [SerializeField] private Transform[] referenceWaypoints;
        [Tooltip("Ручное значение L_эт в метрах (для режима Manual).")]
        [SerializeField] private float manualReferenceLength;

        [Header("Отчёт")]
        [SerializeField] private string reportFileName = "RobotMetricsReport.md";
        [Tooltip("Папка для отчёта. Пусто = <проект>/Reports (в редакторе) или persistentDataPath (в билде).")]
        [SerializeField] private string outputDirectory = "";
        [Tooltip("Переопределить название алгоритма (иначе определяется по типу драйвера).")]
        [SerializeField] private string algorithmNameOverride = "";
        [Tooltip("Переопределить логическое имя сцены (иначе берётся имя сцены без суффикса _Crisp/_Fuzzy).")]
        [SerializeField] private string sceneNameOverride = "";

        [Header("Debug")]
        [SerializeField] private bool debugLog = true;

        [Serializable]
        private class RunRecord
        {
            public string scene;
            public string algorithm;
            public bool goalReached;
            public float timeSeconds;
            public float pathLength;
            public float averageSpeed;
            public int collisions;
            public float referenceLength;
            public float efficiencyPercent;
            public float ise;        // интегральная квадратичная ошибка, м²·с
            public float rmsError;   // СКО поперечного отклонения = sqrt(ISE / T), м
            public string timestampUtc;
        }

        [Serializable]
        private class RunDatabase
        {
            public List<RunRecord> runs = new();
        }

        private IRobotDriver _driver;
        private Rigidbody _rb;

        private bool _started;
        private bool _finalized;
        private float _time;
        private float _pathLength;
        private int _collisions;
        private Vector3 _lastPos;
        private Vector3 _sceneStartPos;
        private Collider _lastCollisionCollider;
        private float _lastCollisionTime;

        // ISE (Integral Square Error): Σ e(t)²·Δt, где e(t) — поперечное отклонение
        // робота от эталонного маршрута (cross-track error) в момент t.
        private float _ise;
        private readonly List<Vector3> _referencePath = new();

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void AutoAttach()
        {
            IRobotDriver driver = FindActiveDriver();
            if (driver is Component driverComponent &&
                driverComponent.GetComponent<RobotRunMetrics>() == null)
            {
                driverComponent.gameObject.AddComponent<RobotRunMetrics>();
            }
        }

        private static IRobotDriver FindActiveDriver()
        {
            CrispRobotDriver crisp = FindFirstObjectByType<CrispRobotDriver>();
            if (crisp != null && crisp.isActiveAndEnabled)
            {
                return crisp;
            }

            FuzzyRobotDriver fuzzy = FindFirstObjectByType<FuzzyRobotDriver>();
            if (fuzzy != null && fuzzy.isActiveAndEnabled)
            {
                return fuzzy;
            }

            if (crisp != null)
            {
                return crisp;
            }

            return fuzzy;
        }

        private void Awake()
        {
            _rb = GetComponent<Rigidbody>();
            _driver = ResolveDriver();

            _sceneStartPos = Flatten(_rb != null ? _rb.position : transform.position);
            _lastPos = _sceneStartPos;
        }

        private IRobotDriver ResolveDriver()
        {
            CrispRobotDriver crisp = GetComponent<CrispRobotDriver>();
            if (crisp != null && crisp.enabled)
            {
                return crisp;
            }

            FuzzyRobotDriver fuzzy = GetComponent<FuzzyRobotDriver>();
            if (fuzzy != null && fuzzy.enabled)
            {
                return fuzzy;
            }

            return (IRobotDriver)crisp ?? fuzzy;
        }

        private void FixedUpdate()
        {
            if (_finalized || _rb == null)
            {
                return;
            }

            Vector3 pos = Flatten(_rb.position);
            float speed = Flatten(_rb.linearVelocity).magnitude;

            if (!_started)
            {
                if (startOnFirstMovement && speed < movementThreshold)
                {
                    // Робот ещё не тронулся — держим точку старта актуальной.
                    _lastPos = pos;
                    return;
                }

                _started = true;
                _lastPos = pos;
                BuildReferencePath();

                if (debugLog)
                {
                    Debug.Log("[Metrics] Старт измерения прохождения сцены.");
                }
            }

            float dt = Time.fixedDeltaTime;

            float moved = Vector3.Distance(pos, _lastPos);
            if (moved > pathDeadband)
            {
                _pathLength += moved;
            }

            // ISE: накапливаем квадрат поперечного отклонения от эталонного маршрута.
            float error = CrossTrackError(pos);
            _ise += error * error * dt;

            _lastPos = pos;
            _time += dt;

            if (_driver != null && _driver.HasReachedTarget)
            {
                Finalize(true);
            }
        }

        private void OnCollisionEnter(Collision collision)
        {
            CountCollision(collision.collider);
        }

        private void CountCollision(Collider other)
        {
            if (_finalized || !_started || other == null)
            {
                return;
            }

            int layer = other.gameObject.layer;
            if ((obstacleLayers.value & (1 << layer)) == 0)
            {
                return; // не препятствие (земля/цель/прочее)
            }

            float now = Time.time;
            if (other == _lastCollisionCollider && now - _lastCollisionTime < collisionDebounce)
            {
                return; // тот же контакт — не считаем повторно
            }

            _lastCollisionCollider = other;
            _lastCollisionTime = now;
            _collisions++;

            if (debugLog)
            {
                Debug.Log($"[Metrics] Столкновение #{_collisions} с «{other.name}».");
            }
        }

        private void OnApplicationQuit()
        {
            if (!_finalized && _started)
            {
                Finalize(_driver != null && _driver.HasReachedTarget);
            }
        }

        private void OnDisable()
        {
            // Выход из Play Mode в редакторе / выгрузка сцены.
            if (!_finalized && _started)
            {
                Finalize(_driver != null && _driver.HasReachedTarget);
            }
        }

        private void Finalize(bool reachedGoal)
        {
            if (_finalized)
            {
                return;
            }

            _finalized = true;

            float referenceLength = ComputeReferenceLength();
            float averageSpeed = _time > 1e-4f ? _pathLength / _time : 0f;
            float efficiency = _pathLength > 1e-4f ? referenceLength / _pathLength * 100f : 0f;
            float rmsError = _time > 1e-4f ? Mathf.Sqrt(_ise / _time) : 0f;

            var record = new RunRecord
            {
                scene = ResolveSceneName(),
                algorithm = ResolveAlgorithmName(),
                goalReached = reachedGoal,
                timeSeconds = _time,
                pathLength = _pathLength,
                averageSpeed = averageSpeed,
                collisions = _collisions,
                referenceLength = referenceLength,
                efficiencyPercent = efficiency,
                ise = _ise,
                rmsError = rmsError,
                timestampUtc = DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss 'UTC'", CultureInfo.InvariantCulture)
            };

            if (debugLog)
            {
                Debug.Log(
                    $"[Metrics] Итог [{record.algorithm} / {record.scene}]: цель={(reachedGoal ? "да" : "нет")}, " +
                    $"T={record.timeSeconds:F2} c, L_факт={record.pathLength:F2} м, v_ср={record.averageSpeed:F2} м/с, " +
                    $"столкновений={record.collisions}, L_эт={record.referenceLength:F2} м, η={record.efficiencyPercent:F1}%, " +
                    $"ISE={record.ise:F3} м²·с, СКО={record.rmsError:F3} м");
            }

            WriteReport(record);
        }

        private float ComputeReferenceLength()
        {
            return referenceMode switch
            {
                ReferenceMode.Manual => Mathf.Max(0f, manualReferenceLength),
                ReferenceMode.Waypoints => ComputeWaypointLength(),
                _ => StraightLineLength()
            };
        }

        private float StraightLineLength()
        {
            Vector3 goal = _driver != null && _driver.Target != null
                ? Flatten(_driver.Target.position)
                : _sceneStartPos;

            return (goal - _sceneStartPos).magnitude;
        }

        private float ComputeWaypointLength()
        {
            Vector3 prev = _sceneStartPos;
            float total = 0f;

            if (referenceWaypoints != null)
            {
                foreach (Transform wp in referenceWaypoints)
                {
                    if (wp == null)
                    {
                        continue;
                    }

                    Vector3 point = Flatten(wp.position);
                    total += (point - prev).magnitude;
                    prev = point;
                }
            }

            if (_driver != null && _driver.Target != null)
            {
                total += (Flatten(_driver.Target.position) - prev).magnitude;
            }

            // Если точки не заданы — это просто прямая старт→цель.
            return total;
        }

        /// <summary>
        /// Строит эталонный маршрут для расчёта поперечной ошибки (ISE):
        /// старт → [точки центральной линии, если заданы] → цель.
        /// </summary>
        private void BuildReferencePath()
        {
            _referencePath.Clear();
            _referencePath.Add(_sceneStartPos);

            if (referenceMode == ReferenceMode.Waypoints && referenceWaypoints != null)
            {
                foreach (Transform wp in referenceWaypoints)
                {
                    if (wp != null)
                    {
                        _referencePath.Add(Flatten(wp.position));
                    }
                }
            }

            if (_driver != null && _driver.Target != null)
            {
                _referencePath.Add(Flatten(_driver.Target.position));
            }
        }

        /// <summary>
        /// e(t) для ISE — кратчайшее расстояние от текущей позиции робота до эталонного маршрута
        /// (разница между «желаемым» положением на маршруте и реальным положением робота).
        /// </summary>
        private float CrossTrackError(Vector3 position)
        {
            int count = _referencePath.Count;
            if (count == 0)
            {
                return 0f;
            }

            if (count == 1)
            {
                return (position - _referencePath[0]).magnitude;
            }

            float best = float.PositiveInfinity;
            for (int i = 0; i + 1 < count; i++)
            {
                float d = DistancePointToSegment(position, _referencePath[i], _referencePath[i + 1]);
                if (d < best)
                {
                    best = d;
                }
            }

            return best;
        }

        private static float DistancePointToSegment(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float lengthSq = ab.sqrMagnitude;
            if (lengthSq < 1e-8f)
            {
                return (p - a).magnitude;
            }

            float t = Mathf.Clamp01(Vector3.Dot(p - a, ab) / lengthSq);
            Vector3 projection = a + ab * t;
            return (p - projection).magnitude;
        }

        private string ResolveAlgorithmName()
        {
            if (!string.IsNullOrEmpty(algorithmNameOverride))
            {
                return algorithmNameOverride;
            }

            return _driver switch
            {
                CrispRobotDriver => "Четкий (crisp)",
                FuzzyRobotDriver => "Нечеткий (fuzzy)",
                null => "Неизвестный",
                _ => _driver.GetType().Name
            };
        }

        private string ResolveSceneName()
        {
            if (!string.IsNullOrEmpty(sceneNameOverride))
            {
                return sceneNameOverride;
            }

            string original = gameObject.scene.name;
            string name = original;

            string[] suffixes = { "_Crisp", "_Fuzzy", "-Crisp", "-Fuzzy", " Crisp", " Fuzzy", "Crisp", "Fuzzy" };
            foreach (string suffix in suffixes)
            {
                if (name.EndsWith(suffix, StringComparison.OrdinalIgnoreCase))
                {
                    name = name.Substring(0, name.Length - suffix.Length);
                    break;
                }
            }

            name = name.TrimEnd('_', '-', ' ');
            return string.IsNullOrEmpty(name) ? original : name;
        }

        private void WriteReport(RunRecord record)
        {
            try
            {
                string dir = ResolveOutputDirectory();
                Directory.CreateDirectory(dir);

                string mdPath = Path.Combine(dir, reportFileName);
                string jsonPath = mdPath + ".data.json"; // служебное состояние для накопления прогонов

                RunDatabase db = LoadDatabase(jsonPath);

                // Заменяем предыдущий прогон той же сцены тем же алгоритмом (последний результат — актуальный).
                db.runs.RemoveAll(r => r.scene == record.scene && r.algorithm == record.algorithm);
                db.runs.Add(record);

                File.WriteAllText(jsonPath, JsonUtility.ToJson(db, true), Encoding.UTF8);
                File.WriteAllText(mdPath, BuildMarkdown(db), Encoding.UTF8);

                if (debugLog)
                {
                    Debug.Log($"[Metrics] Отчёт обновлён: {mdPath}");
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[Metrics] Не удалось записать отчёт: {e.Message}");
            }
        }

        private string ResolveOutputDirectory()
        {
            if (!string.IsNullOrEmpty(outputDirectory))
            {
                return outputDirectory;
            }

            return Application.isEditor
                ? Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Assets", "Reports"))
                : Path.Combine(Application.persistentDataPath, "Reports");
        }

        private static RunDatabase LoadDatabase(string jsonPath)
        {
            try
            {
                if (File.Exists(jsonPath))
                {
                    RunDatabase db = JsonUtility.FromJson<RunDatabase>(File.ReadAllText(jsonPath, Encoding.UTF8));
                    if (db != null)
                    {
                        db.runs ??= new List<RunRecord>();
                        return db;
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[Metrics] Не удалось прочитать историю прогонов ({jsonPath}): {e.Message}");
            }

            return new RunDatabase();
        }

        private static string BuildMarkdown(RunDatabase db)
        {
            var sb = new StringBuilder();

            sb.AppendLine("# Сравнение алгоритмов движения робота");
            sb.AppendLine();
            sb.AppendLine("Показатели эффективности (раздел 1.3): достижение цели, время прохождения **T**, ");
            sb.AppendLine("длина траектории **L_факт**, средняя скорость **v_ср = L_факт / T** (формула 2), ");
            sb.AppendLine("число столкновений и коэффициент эффективности маршрута **η = L_эт / L_факт · 100 %** (формула 3).");
            sb.AppendLine();
            sb.AppendLine("Дополнительно — интегральная квадратичная ошибка управления **ISE = Σ e(t)²·Δt**, ");
            sb.AppendLine("где e(t) — поперечное отклонение робота от эталонного маршрута (cross-track error). ");
            sb.AppendLine("**СКО = √(ISE / T)** — то же отклонение в метрах для наглядности.");
            sb.AppendLine();
            sb.AppendLine("> Чем ближе η к 100 % и чем меньше ISE/СКО, тем точнее и аккуратнее движется робот.");
            sb.AppendLine();

            var scenes = new List<string>();
            foreach (RunRecord r in db.runs)
            {
                if (!scenes.Contains(r.scene))
                {
                    scenes.Add(r.scene);
                }
            }

            scenes.Sort(StringComparer.OrdinalIgnoreCase);

            foreach (string scene in scenes)
            {
                sb.AppendLine($"## Сцена: {scene}");
                sb.AppendLine();
                sb.AppendLine("| Алгоритм | Цель достигнута | T, с | L_факт, м | v_ср, м/с | Столкновения | L_эт, м | η, % | ISE, м²·с | СКО, м |");
                sb.AppendLine("|---|:---:|---:|---:|---:|---:|---:|---:|---:|---:|");

                var rows = new List<RunRecord>();
                foreach (RunRecord r in db.runs)
                {
                    if (r.scene == scene)
                    {
                        rows.Add(r);
                    }
                }

                rows.Sort((a, b) => string.Compare(a.algorithm, b.algorithm, StringComparison.OrdinalIgnoreCase));

                foreach (RunRecord r in rows)
                {
                    sb.AppendLine(
                        $"| {r.algorithm} | {(r.goalReached ? "да" : "нет")} | {F2(r.timeSeconds)} | {F2(r.pathLength)} | " +
                        $"{F2(r.averageSpeed)} | {r.collisions} | {F2(r.referenceLength)} | {F1(r.efficiencyPercent)} | " +
                        $"{F3(r.ise)} | {F3(r.rmsError)} |");
                }

                sb.AppendLine();
            }

            sb.AppendLine("---");
            sb.AppendLine($"_Обновлено: {DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss 'UTC'", CultureInfo.InvariantCulture)}_");

            return sb.ToString();
        }

        private static string F2(float value)
        {
            return value.ToString("0.00", CultureInfo.InvariantCulture);
        }

        private static string F1(float value)
        {
            return value.ToString("0.0", CultureInfo.InvariantCulture);
        }

        private static string F3(float value)
        {
            return value.ToString("0.000", CultureInfo.InvariantCulture);
        }

        private static Vector3 Flatten(Vector3 v)
        {
            v.y = 0f;
            return v;
        }
    }
}
