# Публичная среда Mars Rover

Это полный студенческий набор разработки: все публичные тренировочные биомы,
исходники движка, Python-обёртка, GUI, обучение, smoke-тесты и упаковка посылки.
Закрытые тестовые биомы, их параметры, сиды и визуальные профили сюда не входят.

## Сборка

Требуются Python 3.10+, C++20-компилятор, `setuptools`, `wheel` и `pybind11`.

```bash
python -m pip install --upgrade setuptools wheel pybind11
python -m pip install --no-build-isolation --force-reinstall .
```

Проверить установку и увидеть доступные публичные биомы:

```bash
python -c "import _mars_rover_cpp as m; print(m.biome_catalog())"
```

GUI ставится отдельно из вложенного каталога:

```bash
python -m pip install --no-build-isolation --force-reinstall ./gui
mars-rover-play --list-biomes
mars-rover-play --biome gravity_shelf_lug --debug
```

Перед отправкой стартового решения можно проверить весь контракт:

```bash
make check
make submission
```

## Добавление нового тренировочного биома

1. Откройте `cpp/include/mars/custom_biomes.inc.hpp`.
2. Скопируйте `ExampleTrainingBiome`, задайте уникальные `id()` и
   `display_name()` и настройте `sample_params()` и `visuals()`.
3. Оставьте `split()` равным `BiomeSplit::Train`: пользовательские биомы должны
   относиться только к тренировочному каталогу.
4. Создайте статический экземпляр класса в `custom_biomes::append` и добавьте
   его адрес в `out`.
5. Повторите установку пакета и проверьте каталог командой выше.

Для более глубокой механики доступны:

- `cpp/include/mars/biome_bank.hpp` — реестр и параметры биомов;
- `cpp/src/terrain.cpp` — генерация рельефа;
- `cpp/src/mechanics.cpp` — эффекты среды;
- `python/mars_rover_env/configs/env.yaml` — параметры эпизода и генерации;
- `gui/` — визуализация публичного каталога.

Не меняйте контракт действий, наблюдений и конструкцию ровера, если планируете
использовать обученную модель в официальном evaluator.
