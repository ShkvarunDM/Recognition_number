# Recognition number
Распознавание номера партии на изображении с товарной этикеткой, используя библиотеки OpenCV и Tesseract
## ЗАДАНИЕ :pencil:
Разработать прототип системы распознавания номера на товарных этикетках на языке C++, с применением библиотек компьютерного зрения OpenCV и Tesseract. 

### Общие требования к заданию:
1.	Задание выполнять в интегрированной среде разработки Visual Studio на языке программирования высокого уровня C++.
2.	На вход программы подается файл с изображением товарной этикетки в формате .jpg.
3.	На этикетке указана информация о товаре, в частности, 12-тизначный номер партии, выделенный в файле img_test.jpg красной рамкой.
4.	Расположение номера партии на этикетке не изменяется.
5.	Для загрузки и предобработки изображения необходимо применить компо-ненты библиотеки компьютерного зрения OpenCV.
6.	При выполнении задания использовать библиотеку Tesseract для распознавания номера.
7.	Результатом выполнения работы программы является файл с исходным изоб-ражением, переименованный на номер партии.

## Установка 
### Подключение библиотеки OpenCV к проекту Visual Studio

1.	Загрузить OpenCV с https://opencv.org/releases / (Текущая версия: 4.9.0)
2.	Загруженный файл: opencv-4.9.0-windows (Windows 10)
3.	Настройка и извлечение на диске D:\ 
4.	Открыть свой проект на C ++. Щелкаем правой кнопкой мыши и выбираем свойства в основном решении проекта.
5.	Установить конфигурацию в соответствии с рисунком ниже:

   ![image](https://github.com/ShkvarunDM/Recognition_number/assets/103378631/66d82435-ef19-4f29-bac3-12d0db71920c)

6.	Перейти в C / C ++ -> Общие -> Дополнительные каталоги включения

      Добавить:
      `D:\opencv\build\include`
      `D:\opencv\build\include\opencv2`

7.	Компоновщик -> Общие -> Дополнительные зависимости библиотеки

      Добавить:
      `C:\opencv\build\x64\vc16\lib`

8.	Компоновщик -> Ввод -> Дополнительные зависимости
    
      Добавить:
      `opencv_world490.lib`

### Установка Tesseract OCR

1.	Загрузить файл с https://github.com/UB-Mannheim/tesseract/wiki
2.	Установить tesseract-ocr-w64-setup-v5.0.0-alpha.20210506.exe

## Этапы выполнения работы программы
1. Преобразования в градации серого
   
![image](https://github.com/ShkvarunDM/Recognition_number/assets/103378631/6b90213e-3c6c-4d7b-8ef0-5caf06061751)

2. Обнаружение краев изображения
 
![image](https://github.com/ShkvarunDM/Recognition_number/assets/103378631/618d1391-64b0-47f4-bc18-ded4fc3236b4)

3. Закрытие контуров
   
   ![image](https://github.com/ShkvarunDM/Recognition_number/assets/103378631/a7626e38-4c2e-4d6b-bd80-3fcb2d530b32)
   
4. Выделение области этикетки
   
   ![image](https://github.com/ShkvarunDM/Recognition_number/assets/103378631/4d2eb5a5-8756-4f0f-918f-188ed72ecc84)

5. Выделение интересующей области номера
   
   ![image](https://github.com/ShkvarunDM/Recognition_number/assets/103378631/7cbebc78-a07d-4295-9d94-881c62be56c3)

6.	Предобработка области номера до распознания
   
   ![image](https://github.com/ShkvarunDM/Recognition_number/assets/103378631/ef60cc67-64c1-4b54-adc7-b6cbc46af269)

7.	Распознание 12-тизначного кода с применением инструментов библиотеки Tesseract.

   ![image](https://github.com/ShkvarunDM/Recognition_number/assets/103378631/a950e439-c465-4e3e-b8f8-6147a8a684e2)

