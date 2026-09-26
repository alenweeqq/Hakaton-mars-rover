# Student GUI

GUI использует только студенческий каталог: встроенные и train-биомы. Даже если
рядом случайно установлена организаторская сборка среды, каталог фильтруется, а
создаваемая среда принудительно запускается с train-split.

```bash
make build
make play
make play ARGS="--list-biomes"
make play ARGS="--biome gravity_shelf_lug --debug"
```
