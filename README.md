# Fix-RotationKon

Прошивка контролера поворотки (самої поворотки). Отримує команди від Fix-RotationKonWT по UART та керує двигуном.

## Протокол UART з Fix-RotationKonWT

### PACKET_GOTO_AZIMUTH (0x21)

Payload підтримує 2 формати (backward compatible):

- len=2: angle_lo, angle_hi — контролер сам обирає напрямок (старий формат).
- len=3: angle_lo, angle_hi, dir — контролер крутить примусово у вказаному напрямку.

dir:
- 1 — CW (за годинниковою)
- 2 — CCW (проти годинникової)

Якщо dir не задано (старий формат) або невідоме значення — використовується стандартний startAzimuthMove(...) (найкоротший шлях з урахуванням AZIMUTH_ALLOW_ZERO_CROSSING).

## Зміни в логіці руху

Додано startAzimuthMoveForced(target, direction), який рахує бюджет імпульсів саме для CW або CCW і не намагається обирати «коротший» напрямок.

Це потрібно, щоб не проходити через заборонені азімути: вибір напряму робить міст Fix-RotationKonWT на основі allowedFromDeg/allowedToDeg, а поворотка лише виконує заданий напрямок.
