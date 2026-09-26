# Первая обучаемая версия: действие GAS + DOWN

В этой версии **нет встроенной эвристики fix_downshift**: PPO самостоятельно учится выбирать действия. Это экспериментальная замена одного макродействия, а не гарантированное улучшение.

Изменён только `python/mars_rover_env/actions.py`: последнее действие GAS+RIGHT+HEATER заменено на GAS+DOWN. Число действий осталось 31. Старое действие CLUTCH+DOWN сохранено. `model.py` и `train.py` совпадают с присланными исходными файлами. Важно: понижение передачи может требовать отпускания газа/сцепления — успешность нового макродействия ещё не проверена физически.

1. В корне проекта сохранить резервные копии: `Copy-Item .\model.py .\model_before_ppo.py`; `Copy-Item .\train.py .\train_before_ppo.py`; `Copy-Item .\python\mars_rover_env\actions.py .\python\mars_rover_env\actions_before_ppo.py`.
2. Скопировать файлы из архива в соответствующие места проекта с заменой. Не копировать всю папку `python` поверх проекта — только `actions.py`.
3. Проверить: `$env:PYTHONPATH=".;python"`; `.\.venv\Scripts\python.exe -m unittest discover -s tests -v`.
4. Проверить макродействия: `.\.venv\Scripts\python.exe -c "from mars_rover_env.actions import ACTION_MACROS; print(len(ACTION_MACROS), ACTION_MACROS[30], ACTION_MACROS[8]); assert len(ACTION_MACROS)==31 and ACTION_MACROS[30]==129 and ACTION_MACROS[8]==136"`.
5. `.\.venv\Scripts\python.exe .\package_submission.py` и `.\.venv\Scripts\python.exe -m zipfile -l .\submission.zip`.
6. Отправлять только после успешной проверки. Локальный smoke-test не проверяет доступность arena-base, полную сборку Docker или качество обученной политики.

Не нужно запускать train.py локально на Windows: он использует платформенные `arena.protocol` и `vendor.meta_ppo`.
