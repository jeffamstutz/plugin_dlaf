# dlaf OSPRay Studio Plugin

PoC OSPRay Studio plugin to use OSPRay to render data created by the dlaf
implementation initial created by Michael Fogleman (original code found
[here](https://github.com/fogleman/dlaf)).

## Build Instructions

Clone this repository into your OSPRay Studio source tree under:
`ospray_studio/plugins` and enable in cmake with `BUILD_PLUGIN_DLAF`.

## Run Instructions

Run with:

```bash
./ospStudio --plugin dlaf
```

...then find the simulation control panel in the `Panels` menu.

