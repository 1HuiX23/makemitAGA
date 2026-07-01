# Third-Party Notices

MakemitAGA includes, incorporates, or is built with third-party software and font resources.
This notice is a summary only. The complete license texts are provided in the `Licenses/` directory.

Unless stated otherwise, third-party components retain their original copyright and license terms.
MakemitAGA's own license does not replace or override those terms.

---

## CSharpMath

**Components used**

- CSharpMath.SkiaSharp 0.5.1
- CSharpMath.Rendering and other transitive CSharpMath components required by that package

**Purpose in MakemitAGA**

CSharpMath parses and typesets LaTeX-style mathematical expressions. MakemitAGA uses
`CSharpMath.SkiaSharp.MathPainter` to render formulae into transparent images, which are then
loaded into Unity as runtime textures.

MakemitAGA does **not** use CSharpMath.Forms, CSharpMath.Avalonia, WPF-Math, or TEXDraw.

**License**

MIT License.

**Full license text**

- `Licenses/CSharpMath-MIT.txt`

---

## Typography

**Purpose in MakemitAGA**

Typography is used by CSharpMath.Rendering for font lookup, reading, and text/math typesetting.
It is included indirectly through CSharpMath.

**License**

The Typography project is licensed under the MIT License. Some Typography source files may carry
additional permissive notices; the exact upstream license file corresponding to the bundled version
should be retained.

**Full license text**

- `Licenses/Typography-LICENSE.md`

---

## Mathematical fonts distributed through CSharpMath.Rendering

CSharpMath.Rendering includes font resources used for mathematical formula rendering.

### Latin Modern Math

- License: GUST Font License
- Full text: `Licenses/LatinModernMath-GUST-Font-License.txt`

### Cyrillic Modern

- License: SIL Open Font License 1.1
- Full text: `Licenses/OFL-1.1.txt`

### AMS Capital Blackboard Bold

- License: SIL Open Font License 1.1
- Full text: `Licenses/OFL-1.1.txt`

These font licenses apply to the font software bundled through CSharpMath.Rendering. They do not
apply to MiSide's own runtime fonts. MakemitAGA does not redistribute MiSide's font files; normal
dialogue text references the font already loaded by the game at runtime.

---

## SkiaSharp

**Components used**

- SkiaSharp 2.88.9
- SkiaSharp.NativeAssets.Win32 2.88.9
- Native `libSkiaSharp` for Windows x64

**Purpose in MakemitAGA**

SkiaSharp rasterizes CSharpMath's formula layout into transparent bitmap data.

**License**

SkiaSharp is licensed under the MIT License.

**Full license and dependency notices**

- `Licenses/SkiaSharp-MIT.txt`
- `Licenses/Skia-BSD-3-Clause.txt`
- `Licenses/SkiaSharp-External-Dependency-Info-2.88.9.txt`

The external dependency information file must come from the same SkiaSharp version used by the
project.

---

## Fody

**Version used**

- Fody 6.8.2

**Purpose in MakemitAGA**

Build-time assembly weaving.

**License**

MIT License.

**Full license text**

- `Licenses/Fody-MIT.txt`

---

## Costura.Fody

**Version used**

- Costura.Fody 6.0.0

**Purpose in MakemitAGA**

Embeds selected managed NuGet dependencies into `MakemitAGA.dll` and injects dependency-loading
support code.

**License**

MIT License.

**Full license text**

- `Licenses/Costura-MIT.txt`

---

## NeuroMita / MitaAI-derived material

MakemitAGA contains, and may continue to incorporate, code,
implementation techniques, architectural patterns, and adapted logic
from [NeuroMita](https://github.com/VinerX/NeuroMita), created by VinerX.

The referenced or adapted material may appear across multiple subsystems,
including, but not limited to, IL2CPP integration, AssetBundle loading,
animation and character control, Harmony patches, dialogue systems,
visual perception, navigation, interaction logic, and AI-related features.

Where NeuroMita-derived material is used, the original copyright,
attribution requirements, and license terms remain in effect.

MakemitAGA claims copyright only over its original code and modifications.
It does not claim ownership of the original NeuroMita material or replace
its license terms.

See:

- `Licenses/NeuroMita-LICENSE.md`

---

## Components referenced but not redistributed

MakemitAGA builds against or interacts with the following software, but the project repository and
release archive should not redistribute their binaries unless their licenses and redistribution
requirements are handled separately:

- MiSide and its game assets, fonts, models, textures, audio, and assemblies
- Unity Engine assemblies
- BepInEx assemblies
- Il2CppInterop assemblies
- Harmony assemblies
- Assembly-CSharp and other game-generated interop assemblies

MakemitAGA is an unofficial project and is not affiliated with or endorsed by the developers or
publishers of MiSide. Users must obtain MiSide separately.

---

## Python backend runtime dependencies

The source-only Python backend uses the following runtime dependency:

- Requests — Apache License 2.0

The dependency itself is not vendored or redistributed in this repository.
Users install it separately through pip.

Additional Python, Nuitka, and bundled dependency notices will be included
if a compiled OnlineAIApiServer.exe is distributed in a future release.