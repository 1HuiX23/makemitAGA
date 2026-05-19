# makemitAGA
<div align="center">

# MakemitAGA - MiSide AI Interaction Mod

[**English**](#english-version) | [**Русский**](#russian-version) | [**中文**](#chinese-version)

</div>

---

<a name="english-version"></a>
## 🇬🇧 English Version

MakemitAGA is an **AI dialogue and behavior control plugin** developed for the 3D game *MiSide*. Based on the BepInEx (IL2CPP) framework, this project aims to explore the integration of Multimodal Large Language Models (VLM) with the game's environment physics and animation system (FinalIK), attempting to give the character "Mita" a more dynamic way to interact with the environment.

> ⚠️ **Note on Code Generation (Vibe Coding)**  
> The core architecture and most of the code in this project were generated with AI assistance (Vibe Coding). This approach allowed us to rapidly explore the application of Large Models at the game engine level. Therefore, some non-traditional implementations may exist in the codebase.

### 🛠️ Current Features
This project is currently in the early technical verification stage. We are experimenting with the following mechanics:
* **🗣️ LLM Integration & Command Driven:** Built an HTTP architecture between the C# client and Python server. AI-generated text is dynamically converted into 3D physical text in the game, dropping with Rigidbody physics after being read (Text Rain Effect).
* **👁️ Ghost Camera Snapshot Vision:** Addressed visual coordinate misalignment caused by head animations by implementing a background "Ghost Camera" that instantly freezes the perspective for screenshots. Uses Raycast algorithms to map AI-output 2D coordinates to 3D objects.
* **🚶 Native Logic Override & Dynamic Pathfinding:** Uses Harmony patches to intercept the game's native "Magnet" logic. Introduced a pathfinding algorithm based on `NavMesh.SamplePosition` to help the AI navigate the room more smoothly.
* **🪑 FinalIK-based Adaptive Interaction:** We are exploring smooth IK blending techniques to calculate seat height dynamically via Raycast. Combined with `FullBodyBipedIK`, we are attempting to achieve better pelvis height adaptation and foot alignment, striving for more natural character movements.
* **🌆 Environment Control:** Allows the AI to dynamically change global lighting, time, glitch effects, blood visuals, and TV states by parsing text commands.

### 🤝 Acknowledgments & Credits
In the early stages of development, this project drew massive inspiration and help from the [**NeuroMita**](https://github.com/VinerX/NeuroMita) project. NeuroMita provided us with invaluable references for AssetBundle loading, native animation controller overriding, and basic Harmony interception in an IL2CPP environment.

We would like to express our most sincere gratitude to **VinerX**, the author of NeuroMita, and their open-source spirit! We asked for permission before using their code as a reference and received this encouraging reply:
> *"Hello! I'm glad to hear that the code from my project has been useful. You are free to use all the necessary parts of the mod, just, as you planned, please attribute the authorship and include a link to the repository. I wish you success with your own mod!"* — VinerX

**If you are interested in MiSide AI Mods, please be sure to follow and support the [NeuroMita](https://github.com/VinerX/NeuroMita) project!**

### 📜 Disclaimer
1. This project is for learning, technical exchange, and AI interaction testing purposes only. It is **completely free and non-profit**.
2. This project is a third-party player modification (Mod) for *MiSide* and has no official affiliation with the game developer **Aihasto**.
3. Any calls to native APIs and components in the code are runtime memory modifications only. This project **does NOT contain or distribute** any cracked game DLLs or encrypted assets.
4. **If the original game developer (Aihasto), related plugin copyright holders, or the author of NeuroMita believe this project infringes on any rights, please leave a message in the Issues or contact me directly, and I will take it down and cooperate immediately.**

---

<a name="russian-version"></a>
## 🇷🇺 Русская Версия

MakemitAGA — это **плагин для управления диалогами и поведением на базе ИИ**, разработанный для 3D-игры *MiSide*. Проект основан на фреймворке BepInEx (IL2CPP) и направлен на исследование интеграции мультимодальных больших языковых моделей (VLM) с физикой игрового окружения и системой анимации (FinalIK), в попытке дать персонажу "Мита" (Mita) более динамичный способ взаимодействия с миром.

> ⚠️ **Примечание о генерации кода (Vibe Coding)**  
> Основная архитектура и большая часть кода были сгенерированы с помощью ИИ (Vibe Coding). Такой подход позволил нам быстро исследовать применение ИИ на уровне движка игры. Поэтому в коде могут присутствовать нестандартные программные решения.

### 🛠️ Текущие возможности
Проект находится на ранней стадии технической проверки. Мы экспериментируем со следующими механиками:
* **🗣️ Интеграция LLM и управление командами:** Создана HTTP-архитектура для связи C#-клиента и Python-сервера. Текст ИИ динамически преобразуется в 3D физический текст, который падает после прочтения (эффект "дождя из текста").
* **👁️ Визуальное восприятие через призрачную камеру:** Решена проблема смещения координат при анимации головы путем создания невидимой "призрачной камеры" для мгновенных снимков. Алгоритмы Raycast отображают 2D-координаты ИИ на 3D-объекты.
* **🚶 Переопределение нативной логики и поиск пути:** С помощью Harmony перехватывается нативная функция "магнита" (Magnet). Внедрен алгоритм поиска пути на базе `NavMesh.SamplePosition`, чтобы помочь ИИ более плавно перемещаться по комнате.
* **🪑 Адаптивные взаимодействия на базе FinalIK:** Мы исследуем методы плавного смешивания весов IK (Smooth IK Blending) для динамического расчета высоты сиденья через Raycast. В сочетании с `FullBodyBipedIK` мы пытаемся добиться лучшей адаптации высоты таза и выравнивания стоп, стремясь к более естественным движениям персонажа.
* **🌆 Управление окружением:** Позволяет ИИ контролировать освещение, время, эффекты глитча и состояние телевизора через текстовые команды.

### 🤝 Благодарности
На ранних этапах этот проект получил огромное вдохновение от проекта [**NeuroMita**](https://github.com/VinerX/NeuroMita). NeuroMita предоставил нам бесценный пример загрузки AssetBundle в среде IL2CPP, переопределения контроллеров анимации и перехвата методов через Harmony.

Мы хотим выразить искреннюю благодарность автору NeuroMita, **VinerX**, за дух открытого исходного кода! Перед началом работы мы запросили разрешение на использование кода в качестве референса и получили этот вдохновляющий ответ:
> *"Hello! I'm glad to hear that the code from my project has been useful. You are free to use all the necessary parts of the mod, just, as you planned, please attribute the authorship and include a link to the repository. I wish you success with your own mod!"* — VinerX

**Если вы интересуетесь AI-модами для MiSide, обязательно подпишитесь и поддержите проект [NeuroMita](https://github.com/VinerX/NeuroMita)!**

### 📜 Отказ от ответственности (Disclaimer)
1. Этот проект предназначен исключительно для обучения, технического обмена и тестирования взаимодействия с ИИ. Проект **полностью бесплатен и некоммерческий**.
2. Данный проект является сторонней модификацией (Mod) и не имеет официальной связи с разработчиком игры **Aihasto**.
3. Любые вызовы нативных API осуществляются только в оперативной памяти во время игры. Проект **НЕ содержит и НЕ распространяет** взломанные DLL-файлы игры.
4. **Если разработчик оригинальной игры (Aihasto), правообладатели плагинов или автор NeuroMita сочтут, что этот проект нарушает какие-либо права, пожалуйста, оставьте сообщение в разделе Issues или свяжитесь со мной, и я немедленно удалю проект.**

---

<a name="chinese-version"></a>
## 🇨🇳 中文版本 (Chinese Version)

MakemitAGA 是为 3D 游戏《MiSide》开发的一款**多模态 AI 交互与行为控制插件**。本项目基于 BepInEx (IL2CPP) 框架，旨在探索将多模态大模型（VLM）与游戏内的环境物理、动画系统（FinalIK）相结合，尝试赋予游戏角色“米塔（Mita）”更加动态的环境交互能力。

> ⚠️ **关于代码生成的说明 (Vibe Coding)**  
> 本项目的核心架构设计与绝大部分代码均由 AI 辅助生成（Vibe Coding）。这种开发方式让我们得以快速探索大模型在游戏引擎底层的应用。因此，代码中可能存在一些非传统的实现方式。

### 🛠️ 目前的探索进度
本项目目前正处于早期的技术验证阶段，我们正在尝试验证以下机制：
* **🗣️ 大模型对话与指令驱动：** 构建了 C# 客户端与 Python 后端的 HTTP 通信架构。将 AI 生成的文本在游戏中动态转化为 3D 物理文字，并在阅读后伴随 Rigidbody 物理效果掉落（文字雨特效）。
* **👁️ 幽灵相机视觉感知：** 尝试解决因头部动画导致 AI 视觉坐标错位的问题，实现了后台“幽灵相机”瞬间冻结视角并截图。通过 Raycast 算法将 AI 输出的 2D 坐标映射到 3D 游戏物体上。
* **🚶 逻辑拦截与动态寻路：** 通过 Harmony 拦截了游戏原生的“磁吸（Magnet）”逻辑。引入了基于 `NavMesh.SamplePosition` 的寻路算法，帮助 AI 更平滑地在房间内移动。
* **🪑 基于 FinalIK 的交互优化：** 我们正尝试使用平滑权重过渡（Smooth IK Blending）技术，利用射线探测动态计算座面高度。结合 `FullBodyBipedIK`，希望能实现更好的盆骨高度自适应与脚掌贴地，努力追求更自然的角色动作表现。
* **🌆 游戏环境与特效控制：** 允许 AI 通过解析文本指令，动态接管游戏的全局光照、时间、花屏特效、流血视觉以及电视机状态等。

### 🤝 鸣谢
本项目在开发初期，从 [**NeuroMita**](https://github.com/VinerX/NeuroMita) 项目中获得了巨大的启发和帮助。NeuroMita 为我们在 IL2CPP 环境下的 AssetBundle 加载、原生动画控制器覆盖以及基础的 Harmony 拦截上提供了极为宝贵的参考。

我们要向 NeuroMita 的作者 **VinerX** 及其开源精神表达最诚挚的感谢！在开发前，我们向作者征询了代码参考的许可，并收到了令人振奋的回复：
> *"Hello! I'm glad to hear that the code from my project has been useful. You are free to use all the necessary parts of the mod, just, as you planned, please attribute the authorship and include a link to the repository. I wish you success with your own mod!"* — VinerX

**如果您对 MiSide 的 AI Mod 感兴趣，请务必去关注和支持 [NeuroMita](https://github.com/VinerX/NeuroMita) 项目！**

### 📜 免责声明
1. 本项目仅供学习、技术交流与 AI 交互行为测试使用，**完全免费且非盈利**。
2. 本项目属于《MiSide》的第三方玩家模组（Mod），与原游戏开发商 **Aihasto** 无任何官方关联。
3. 代码中包含的任何对游戏原生 API 的调用仅为运行时内存修改，本项目**不包含、不分发**任何游戏原生的破解 DLL 文件或加密资产。
4. **如果原游戏开发者（Aihasto）、相关插件版权方或 NeuroMita 作者认为本项目存在任何侵权或不妥之处，请直接在 Issue 中留言或联系我，我将会在第一时间予以删除并配合处理。**