# ПИД Регулятор
Эти файлы содержат реализацию ПИД регулятора. Эта реализация не зависит от платформы, можно использовать любой МК. 

### Создание
Сначала создается структура PID.
Для инициализации желательно использоавть функцию PID_setPID_t_m(PID *,float, float, float,float,int). Здесь устанавливаются коэффециенты, время между измерениями и режим.

### Режимы:
Последний параметр в функции инициализации 
- 0 - Классический алгоритм
- 1 - Модификация 1 
- 2 - Модификация 2
- 3 - Модифмкация 3
### Использование
Для получения значения сигнала используется функция PID_get(PID *), перед получением нужно выполнить вычисление функцией PID_update(PID *).
