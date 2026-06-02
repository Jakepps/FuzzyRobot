using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace FuzzyRobot
{
    public class RobotTelemetryHud : MonoBehaviour
    {
        private const float MaxIllumination = 1000f;

        [Header("Refs")]
        [SerializeField] private FuzzyRobotDriver driver;
        [SerializeField] private Rigidbody robotRigidbody;
        [SerializeField] private RobotBattery battery;
        [SerializeField] private RobotSurfaceProbe surfaceProbe;
        [SerializeField] private RobotIlluminationProbe illuminationProbe;

        [Header("Presentation")]
        [SerializeField] private Vector2 anchoredPosition = new(18f, -18f);
        [SerializeField] private Vector2 panelSize = new(350f, 238f);
        [SerializeField] private float updateInterval = 0.1f;

        private TMP_Text _speedValue;
        private TMP_Text _batteryValue;
        private TMP_Text _statusValue;
        private TMP_Text _targetValue;
        private TMP_Text _surfaceValue;
        private TMP_Text _illuminationValue;
        private Image _batteryFill;
        private Image _speedFill;
        private Image _illuminationFill;
        private float _nextUpdateTime;

        // Драйвер любого типа (нечеткий/четкий) через общий интерфейс.
        private IRobotDriver _driver;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void CreateDefaultHud()
        {
            if (FindFirstObjectByType<RobotTelemetryHud>() != null)
            {
                return;
            }

            if (FindActiveDriver() == null)
            {
                return;
            }

            GameObject hudObject = new("Robot Telemetry HUD", typeof(RectTransform));
            hudObject.AddComponent<RobotTelemetryHud>();
            // Привязка драйвера и источников телеметрии выполняется в Awake().
        }

        /// <summary>
        /// Выбираем активный драйвер: сначала включённый четкий, затем включённый нечеткий,
        /// иначе любой найденный. Так HUD показывает данные именно того алгоритма, что работает.
        /// </summary>
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
            // driver — сериализованная ссылка из инспектора (нечеткий драйвер, обратная совместимость).
            // Если её нет или она «сломалась» (например, на четкой сцене драйвер заменён) — ищем активный.
            if (_driver == null)
            {
                _driver = driver != null && driver.isActiveAndEnabled
                    ? driver
                    : FindActiveDriver();
            }

            ResolveTelemetrySourcesIfNeeded();
            BuildUi();
        }

        /// <summary>
        /// Подтягиваем источники телеметрии (Rigidbody/батарея/поверхность/свет) с объекта робота,
        /// если они не назначены вручную. Делает HUD самодостаточным на любой сцене.
        /// </summary>
        private void ResolveTelemetrySourcesIfNeeded()
        {
            if (_driver is not Component driverComponent)
            {
                return;
            }

            if (robotRigidbody == null)
            {
                robotRigidbody = driverComponent.GetComponent<Rigidbody>();
            }

            if (battery == null)
            {
                battery = driverComponent.GetComponentInChildren<RobotBattery>();
            }

            if (surfaceProbe == null)
            {
                surfaceProbe = driverComponent.GetComponentInChildren<RobotSurfaceProbe>();
            }

            if (illuminationProbe == null)
            {
                illuminationProbe = driverComponent.GetComponentInChildren<RobotIlluminationProbe>();
            }
        }

        private void Update()
        {
            if (Time.unscaledTime < _nextUpdateTime)
            {
                return;
            }

            _nextUpdateTime = Time.unscaledTime + Mathf.Max(0.02f, updateInterval);
            RefreshValues();
        }

        private void BuildUi()
        {
            Canvas canvas = gameObject.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.sortingOrder = 100;

            CanvasScaler scaler = gameObject.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 0.5f;

            gameObject.AddComponent<GraphicRaycaster>();

            RectTransform root = canvas.GetComponent<RectTransform>();
            root.anchorMin = Vector2.zero;
            root.anchorMax = Vector2.one;

            Image panel = CreateImage("Panel", root, new Color(0.035f, 0.043f, 0.05f, 0.86f));
            RectTransform panelRect = panel.rectTransform;
            panelRect.anchorMin = new Vector2(0f, 1f);
            panelRect.anchorMax = new Vector2(0f, 1f);
            panelRect.pivot = new Vector2(0f, 1f);
            panelRect.anchoredPosition = anchoredPosition;
            panelRect.sizeDelta = panelSize;

            VerticalLayoutGroup panelLayout = panel.gameObject.AddComponent<VerticalLayoutGroup>();
            panelLayout.padding = new RectOffset(16, 16, 14, 14);
            panelLayout.spacing = 9f;
            panelLayout.childControlWidth = true;
            panelLayout.childControlHeight = true;
            panelLayout.childForceExpandWidth = true;
            panelLayout.childForceExpandHeight = false;

            TMP_Text title = CreateText("Title", panelRect, "Телеметрия робота", 21f, FontStyles.Bold, TextAlignmentOptions.Left);
            title.color = new Color(0.9f, 0.96f, 1f, 1f);
            SetLayout(title.gameObject, 0f, 30f);

            _batteryValue = CreateMetric(panelRect, "Батарея", new Color(0.18f, 0.82f, 0.45f, 1f), out _batteryFill);
            _speedValue = CreateMetric(panelRect, "Скорость", new Color(0.2f, 0.64f, 1f, 1f), out _speedFill);
            _illuminationValue = CreateMetric(panelRect, "Свет", new Color(1f, 0.78f, 0.18f, 1f), out _illuminationFill);

            _statusValue = CreateCompactRow(panelRect, "Статус");
            _targetValue = CreateCompactRow(panelRect, "Цель");
            _surfaceValue = CreateCompactRow(panelRect, "Поверхность");

            RefreshValues();
        }

        private TMP_Text CreateMetric(RectTransform parent, string label, Color fillColor, out Image fill)
        {
            GameObject row = CreateRow(parent, $"{label} Row", 34f);

            TMP_Text labelText = CreateText("Label", row.transform, label, 14f, FontStyles.Bold, TextAlignmentOptions.Left);
            labelText.color = new Color(0.68f, 0.75f, 0.8f, 1f);
            SetRowChild(labelText.rectTransform, 92f, 0f, 1f);

            Image track = CreateImage("Track", row.transform, new Color(0.16f, 0.19f, 0.22f, 1f));
            SetRowChild(track.rectTransform, 0f, 1f, 1f);

            fill = CreateImage("Fill", track.rectTransform, fillColor);
            fill.rectTransform.anchorMin = Vector2.zero;
            fill.rectTransform.anchorMax = new Vector2(1f, 1f);
            fill.rectTransform.offsetMin = Vector2.zero;
            fill.rectTransform.offsetMax = Vector2.zero;

            TMP_Text value = CreateText("Value", row.transform, "--", 15f, FontStyles.Bold, TextAlignmentOptions.Right);
            value.color = Color.white;
            SetRowChild(value.rectTransform, 92f, 0f, 1f);

            return value;
        }

        private TMP_Text CreateCompactRow(RectTransform parent, string label)
        {
            GameObject row = CreateRow(parent, $"{label} Row", 24f);

            TMP_Text labelText = CreateText("Label", row.transform, label, 13f, FontStyles.Bold, TextAlignmentOptions.Left);
            labelText.color = new Color(0.64f, 0.7f, 0.76f, 1f);
            SetRowChild(labelText.rectTransform, 108f, 0f, 1f);

            TMP_Text value = CreateText("Value", row.transform, "--", 14f, FontStyles.Normal, TextAlignmentOptions.Right);
            value.color = new Color(0.92f, 0.96f, 1f, 1f);
            SetRowChild(value.rectTransform, 0f, 1f, 1f);

            return value;
        }

        private static GameObject CreateRow(RectTransform parent, string name, float height)
        {
            GameObject row = new(name, typeof(RectTransform), typeof(HorizontalLayoutGroup));
            row.transform.SetParent(parent, false);
            SetLayout(row, 0f, height);

            HorizontalLayoutGroup layout = row.GetComponent<HorizontalLayoutGroup>();
            layout.spacing = 8f;
            layout.childAlignment = TextAnchor.MiddleLeft;
            layout.childControlWidth = true;
            layout.childControlHeight = true;
            layout.childForceExpandWidth = false;
            layout.childForceExpandHeight = true;

            return row;
        }

        private static TMP_Text CreateText(
            string name, Transform parent, 
            string value, float size, 
            FontStyles style, TextAlignmentOptions alignment)
        {
            GameObject textObject = new(name, typeof(RectTransform), typeof(TextMeshProUGUI));
            textObject.transform.SetParent(parent, false);

            TMP_Text text = textObject.GetComponent<TMP_Text>();
            text.text = value;
            text.fontSize = size;
            text.fontStyle = style;
            text.alignment = alignment;
            text.textWrappingMode = TextWrappingModes.NoWrap;

            return text;
        }

        private static Image CreateImage(string name, Transform parent, Color color)
        {
            GameObject imageObject = new(name, typeof(RectTransform), typeof(Image));
            imageObject.transform.SetParent(parent, false);

            Image image = imageObject.GetComponent<Image>();
            image.color = color;
            return image;
        }

        private static void SetLayout(GameObject target, float preferredWidth, float preferredHeight)
        {
            LayoutElement layout = target.GetComponent<LayoutElement>();
            if (layout == null)
            {
                layout = target.AddComponent<LayoutElement>();
            }

            if (preferredWidth > 0f)
            {
                layout.preferredWidth = preferredWidth;
            }

            if (preferredHeight > 0f)
            {
                layout.preferredHeight = preferredHeight;
            }
        }

        private static void SetRowChild(RectTransform target, float preferredWidth, float flexibleWidth, float flexibleHeight)
        {
            LayoutElement layout = target.gameObject.AddComponent<LayoutElement>();
            if (preferredWidth > 0f)
            {
                layout.preferredWidth = preferredWidth;
            }

            layout.flexibleWidth = flexibleWidth;
            layout.flexibleHeight = flexibleHeight;
        }

        private void RefreshValues()
        {
            float speed = robotRigidbody != null ? Flatten(robotRigidbody.linearVelocity).magnitude : 0f;
            float batteryPercent = battery != null ? battery.ChargePercent : 0f;
            float illumination = illuminationProbe != null ? illuminationProbe.CurrentIllumination : 0f;
            float surface = surfaceProbe != null ? surfaceProbe.CurrentSurfaceValue : 0f;

            _speedValue.text = $"{speed:F1} m/s";
            _batteryValue.text = battery != null ? $"{batteryPercent:F0}%" : "--";
            _illuminationValue.text = illuminationProbe != null ? $"{illumination:F0} lx" : "--";

            float maxSpeed = _driver != null ? _driver.MaxSpeedMs : 6f;
            SetFillAmount(_speedFill, speed / Mathf.Max(0.1f, maxSpeed));
            SetFillAmount(_batteryFill, batteryPercent / 100f);
            SetFillAmount(_illuminationFill, illumination / MaxIllumination);

            _batteryFill.color = batteryPercent switch
            {
                < 20f => new Color(0.92f, 0.2f, 0.18f, 1f),
                < 45f => new Color(1f, 0.67f, 0.18f, 1f),
                _ => new Color(0.18f, 0.82f, 0.45f, 1f)
            };

            bool reachedTarget = _driver != null && _driver.HasReachedTarget;
            bool recharging = battery != null && battery.IsRecharging;
            _statusValue.text = reachedTarget ? "Цель достигнута" : recharging ? "Зарядка" : "Движение";
            _targetValue.text = FormatTargetDistance();
            _surfaceValue.text = FormatSurface(surface);
        }

        private string FormatTargetDistance()
        {
            if (_driver == null || _driver.Target == null || robotRigidbody == null)
            {
                return "--";
            }

            Vector3 delta = _driver.Target.position - robotRigidbody.position;
            delta.y = 0f;
            return _driver.HasReachedTarget ? "контакт" : $"{delta.magnitude:F1} m";
        }

        private static string FormatSurface(float surface)
        {
            return surface switch
            {
                >= 1.5f => "Скользкая",
                >= 0.5f => "Шероховатая",
                _ => "Ровная"
            };
        }

        private static Vector3 Flatten(Vector3 value)
        {
            value.y = 0f;
            return value;
        }

        private static void SetFillAmount(Image fill, float amount)
        {
            RectTransform rect = fill.rectTransform;
            rect.anchorMax = new Vector2(Mathf.Clamp01(amount), 1f);
            rect.offsetMax = Vector2.zero;
        }
    }
}
