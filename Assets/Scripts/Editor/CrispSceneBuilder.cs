using System.IO;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace FuzzyRobot.EditorTools
{
    /// <summary>
    /// Утилита для быстрого создания «четкой» сцены сравнения.
    /// Клонирует текущую открытую (нечеткую) сцену и заменяет на роботе
    /// <see cref="FuzzyRobotDriver"/> на <see cref="CrispRobotDriver"/>,
    /// перенося те же настройки. Окружение, цель и старт остаются идентичными —
    /// меняется только алгоритм движения, что и нужно для честного сравнения.
    /// </summary>
    public static class CrispSceneBuilder
    {
        private const string MenuPath = "FuzzyRobot/Создать четкую сцену из текущей";

        [MenuItem(MenuPath)]
        public static void CreateCrispSceneFromCurrent()
        {
            Scene active = SceneManager.GetActiveScene();

            if (string.IsNullOrEmpty(active.path))
            {
                EditorUtility.DisplayDialog(
                    "Четкая сцена",
                    "Сначала откройте и сохраните любую сцену с роботом (нечеткую), затем повторите.",
                    "OK");
                return;
            }

            FuzzyRobotDriver sourceFuzzy = Object.FindFirstObjectByType<FuzzyRobotDriver>(FindObjectsInactive.Include);
            if (sourceFuzzy == null)
            {
                EditorUtility.DisplayDialog(
                    "Четкая сцена",
                    "В текущей сцене не найден FuzzyRobotDriver — нечего заменять.",
                    "OK");
                return;
            }

            // 1) Сохраняем копию активной сцены под новым именем (оригинал не трогаем).
            string dir = Path.GetDirectoryName(active.path);
            string baseName = active.name.Replace("Fuzzy", string.Empty);
            string newPath = AssetDatabase.GenerateUniqueAssetPath(
                Path.Combine(dir, $"{baseName}_Crisp.unity").Replace('\\', '/'));

            if (!EditorSceneManager.SaveScene(active, newPath, true))
            {
                EditorUtility.DisplayDialog("Четкая сцена", "Не удалось сохранить копию сцены.", "OK");
                return;
            }

            // 2) Открываем копию и подменяем драйвер.
            Scene crispScene = EditorSceneManager.OpenScene(newPath, OpenSceneMode.Single);

            FuzzyRobotDriver fuzzy = Object.FindFirstObjectByType<FuzzyRobotDriver>(FindObjectsInactive.Include);
            if (fuzzy == null)
            {
                EditorUtility.DisplayDialog("Четкая сцена", "В копии сцены не найден FuzzyRobotDriver.", "OK");
                return;
            }

            GameObject robot = fuzzy.gameObject;

            CrispRobotDriver crisp = robot.GetComponent<CrispRobotDriver>();
            if (crisp == null)
            {
                crisp = robot.AddComponent<CrispRobotDriver>();
            }

            CopySharedConfig(fuzzy, crisp);

            // Убираем нечеткий драйвер, чтобы остался только четкий.
            // Для prefab-инстанса это корректно записывается как «removed component override».
            try
            {
                Object.DestroyImmediate(fuzzy);
            }
            catch
            {
                // На всякий случай: если удаление недоступно — просто отключаем.
                fuzzy.enabled = false;
            }

            EditorUtility.SetDirty(robot);
            EditorSceneManager.MarkSceneDirty(crispScene);
            EditorSceneManager.SaveScene(crispScene);
            AssetDatabase.Refresh();

            Debug.Log($"[CrispSceneBuilder] Создана сцена «{newPath}». Робот переключён на четкий алгоритм (CrispRobotDriver).");
            EditorUtility.DisplayDialog(
                "Четкая сцена",
                $"Готово!\n\nСоздана сцена:\n{newPath}\n\nРобот использует четкий алгоритм (CrispRobotDriver).\nЗапустите Play для сравнения.",
                "OK");
        }

        /// <summary>
        /// Переносит все сериализуемые настройки (ссылки, лимиты, защитную оболочку и т.п.)
        /// с нечеткого драйвера на четкий по совпадающим именам полей.
        /// </summary>
        private static void CopySharedConfig(FuzzyRobotDriver src, CrispRobotDriver dst)
        {
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

            var dstType = typeof(CrispRobotDriver);

            foreach (FieldInfo sf in typeof(FuzzyRobotDriver).GetFields(flags))
            {
                // Копируем только настройки инспектора, не трогая рантайм-поля.
                bool isSerialized = sf.IsPublic || sf.IsDefined(typeof(SerializeField), true);
                if (!isSerialized || sf.IsInitOnly)
                {
                    continue;
                }

                FieldInfo df = dstType.GetField(sf.Name, flags);
                if (df == null || df.IsInitOnly || df.FieldType != sf.FieldType)
                {
                    continue;
                }

                df.SetValue(dst, sf.GetValue(src));
            }

            EditorUtility.SetDirty(dst);
        }
    }
}
