//  Программа   IGES-SeaHydro_Converter. Версия 1.0.
//  Программа  IGES-SeaHydro_Converter.exe  обрабатывает 3D-геометрию формы корпуса судна из файла типа IGES (расширение .igs)
//  и создаёт новый файл с исходными данными для программы  SeaHydro (расчеты по гидростатике, остойчивости,
//  непотопляемости корабля; разработчик - ООО "Си Тех" www.seatech.ru). 
//  
//
//
#include <windows.h>
#include<iostream>
#include<string>   
#include<conio.h>
#include<vector>
#include <math.h>
//
using namespace std;
//
// Глобальные параметры IGES-файла (из раздела Global Parameter)
struct g_GParameters {
	char GP1_ParameterDelimiter;  // Global Parameter 1 - Parameter Delimiter (Глобальный параметр 1 - Разделитель параметров). По умолчанию = ',' 
	char GP2_RecordDelimiter;  // Global Parameter 2 - Record Delimiter (Глобальный параметр 2 - Разделитель записей). По умолчанию = ';' 
	string GP3_ProductID; // Global Parameter 3 - Product identification from sending system. No default value.
	string GP4_FileName; // Global Parameter 4 - File Name. No default value.
	string GP5_NativeSystemID; // Global Parameter 5 - Native System ID. No default value.
	string GP6_PreprocessorVersion; // Global Parameter 6 - Preprocessor version. No default value.
	int GP7_BitsForInteger; // Global Parameter 7 - Number of Binary Bits for Integer Representation. No default value.
	int GP8_SinglePrecisionMagnitud; // Global Parameter 8 - Single-Precision Magnitud (Maximum power of ten representable in a single-precision floating point number on the sending systeme). No default value.
	int GP9_SinglePrecisionSignificance; // Global Parameter 9 - Single-Precision Significance (Number of significant digits in a single-precision floating point number on the sending system). No default value.
	int GP10_DoublePrecisionMagnitude; // Global Parameter 10 - Double-Precision Magnitude (Maximum power of ten representable in a double-precision floating point number on the sending system). No default value.
	int GP11_DoublePrecisionSignificance; // Global Parameter 11 - Double-Precision Significance (Number of significant digits in a double-precision floating point number on the sending system). No default value.
	string GP12_ProductIdForReceiver; // Global Parameter 12 - Product Identification for the Receiver. The default value is the value specified in parameter 3.
	double GP13_ModelSpaceScale; // Global Parameter 13 - Model Space Scale. Contains the ratio of model space to real-world space. The default value is 1.0.
	int GP14_UnitsFlag; // Global Parameter 14 - Units Flag (Код единицы измерения). default = 1 (inches);  2 - millimeters;  6 - meters.
	string GP15_UnitsName; // Global Parameter 15 - Units Name. String naming the model units in the system; it shall be as field 14 unless field 14 is 3. The default value is 1. Postprocessors shall ignore this field if it is inconsistent with field 14.
	int GP16_MaxNumLineWeightGradations; // Global Parameter 16 - Maximum Number of Line Weight Gradations. The value shall be greater than zero. The default value is 1.
	double GP17_MaximumLineWidth; // Global Parameter 17 - Width of Maximum Line Weight in Units. No default value.
	string GP18_DateTimeFileGeneration; // Global Parameter 18 - Date and Time of Exchange File Generation. 15HYYYYMMDD:HHNNSS  or  13HYYMMDD:HHNNSS  .  No default value.
	double GP19_MinimumResolution; // Global Parameter 19 - Minimum User-Intended Resolution. No default value.
	double GP20_MaxCoordValue; // Global Parameter 20 - Approximate Maximum Coordinate Value. The default value is 0.0.
	string GP21_AuthorName; // Global Parameter 21 - Name of Author.  The default value is NULL, which is interpreted as “unspecified”.
	string GP22_AuthorsOrganization; // Global Parameter 22 - Author’s Organization. The default value is NULL, which is interpreted as “unspecified”.
	int GP23_VersionFlag; // Global Parameter 23 - Version Flag. The default value is 3. Value less than 1 shall assign 3; value greater than 11 shall assign 11.
	int GP24_DraftingStandardFlag; // Global Parameter 24 - Drafting Standard Flag.  The default value is 0.
	string GP25_DateTimeNativModelModified; // Global Parameter 25 - Date and Time Model was Created or Modified. The default value is NULL
	string GP26_AppProtocolIdentifier; // Global Parameter 26 - Application Protocol/Subset Identifier. The default value is NULL.  
};
//
struct Entity124_TransformationMatrix {  // Матрица трансформации (Transformation Matrix Entity - Type 124)
	int D_Section_String_Number;  // Номер строки с данными об этом объекте в Directory Entry Section
	int P_Section_String_Number;  // Номер строки с данными об этом объекте в Parameter Data Section
	int FormNumber;  //  Поле 15 в Directory Entry. 0(default) - "праворукая" система координат. 1 - "леворукая" система координат.
	double R11, R12, R13, T1, R21, R22, R23, T2, R31, R32, R33, T3;
};
//
struct Entity110_Line {  // Информация об элементе "Отрезок" (Entity Type 110 - Line). Цвет, координаты, матрицы трансформации.
	int D_Section_String_Number;  // Номер строки с данными об этом объекте в Directory Entry Section
	int P_Section_String_Number;  // Номер строки с данными об этом объекте в Parameter Data Section
	int Color;  // "Стандарные" цвета 0-8. Нестандартный цвет - 10-значное число формата 1RGB (например для RGB:254,001,220 это 1254001220).
	int FormNumber;  //  Поле 15 в Directory Entry. 0 - отрезок. 1 - луч с началом в (X1,Y1,Z1). 2 - бесконечная прямая
	double X1origin, Y1origin, Z1origin, X2origin, Y2origin, Z2origin;  // X1origin, Y1origin, Z1origin, X2origin, Y2origin, Z2origin - координаты начала и конца отрезка (как записано в IGES-файле, без преобразований)
	double X1, Y1, Z1, X2, Y2, Z2;  // - координаты отрезка в системе координат 3D-модели (после преобразования через матрицы трансформации, т.е. после функции TransformPointByTransfMatrix())
	long long X1_01mm, Y1_01mm, Z1_01mm, X2_01mm, Y2_01mm, Z2_01mm;  // Координаты X1, Y1, Z1, X2, Y2, Z2 после кругления до 0,1мм. Единица измерения - 0,1мм (целое число, чтобы избежать потери точности в дальнейшем)
	long long Length;  // Длина отрезка после преобразования через матрицы трансформации и округления до 0,1мм. Единица измерения - 0,1мм (целое число, чтобы избежать потери точности в дальнейшем)
	vector<Entity124_TransformationMatrix> TransformationMatrixVector; // Вектор с матрицами трансформации (Transformation Matrix Entity - Type 124)
};
//
struct ShipTransformationMatrix {  // Матрица трансформации в судовую систему координат из координат 3D-модели. Получается после анализа положения отрезков осей.
	double R11, R12, R13, T1, R21, R22, R23, T2, R31, R32, R33, T3;
};
//
// Изменение цвета печатаемого текста и фона. Black = 0, Blue = 1, Green = 2, Cyan = 3, Red = 4, Magenta = 5, Brown = 6, LightGray = 7,
//  DarkGray = 8, LightBlue = 9, LightGreen = 10, LightCyan = 11, LightRed = 12, LightMagenta = 13, Yellow = 14, White = 15. 
//  Стандарный цвет текста - SetColor(7,0).
void SetColor(int TextColor, int bg = 0);
//
// Ввод числа типа int с проверкой
int getInt();
//
// Ввод числа типа int с проверкой: возвращаются значения только от Min до Max включительно.
int getInt_WithValueCheck(int Min, int Max);
//
//  Печатает преамбулу 
void PrintPreamble(const char* SourceFileName);
//
// Печатает Справку по программе, создаёт текстовый файл help.txt с текстом справки
void PrintHelp(const char* HelpFileName = "Help.txt");
//
// Ожидает нажатия клавиши ENTER. Все остальные нажатия игнорируются, текст не печатается
void PressENTER();
// 
//  Печатает красным текстом сообщение и после нажатия ENTER выходит из программы 
void PrintErrorAndExit(const string& Text);
//
// Проверяет аргумент на соответствие требованиям к символам для Global Parameter 1 и Global Parameter 2 (по IGES Specification Version 6.0, 2.2.3.1).
// Возвращает true  если символ соответствует, иначе- false.
bool CheckGP1GP2(const char& ch);
//
// Читает в структуру g_GParameters Глобальные Параметры из G_SectionVector, проверяет их по IGES Specification V.6.0. 
// Добавляет информацию о глобальных параметрах в InfoFileText. 
void ReadAndCheckGlobalParameters(const vector<string>& G_SectionVector, string& InfoFileText);
//
// Читает символ разделителя (Parameter Delimiter  или  Record Delimiter) начиная с символа с индексом i в разделах Global 
// Section  и  Parameter Data Section. Устанавливает значение i на символ, следующий за разделителем или за завершающими
// пробелами (если они есть). Аргумент  Section  показывает раздел ('G'- Global Section, 'P'- Parameter Data Section).
// Возвращает прочитанный символ.
char ReadDelimiters(const vector<string>& SectionVector, int& i, const char& Section);
//
// Читает и возвращает параметр типа String, начиная чтение с индекса i в разделах Global Section  и  Parameter Data
// Section. Меняет значение индекса i на символ, следующий за прочитанным текстом. Аргумент  Section  показывает раздел,
// в котором происходит чтение ('G'- Global Section, 'P'- Parameter Data Section).
string Read_String(const vector<string>& SectionVector, const char& GP1_ParameterDelimiter, const char& GP2_RecordDelimiter, int& i, const char& Section);
//
// Читает и возвращает целое число начиная чтение с индекса i в разделах Global Section  и  Parameter Data Section.
//  Меняет значение индекса i на символ, следующий за прочитанным текстом. Аргумент  Section  показывает раздел,
//  в котором происходит чтение ('G'- Global Section, 'P'- Parameter Data Section).
long long  ReadIntegerDataType(const vector<string>& SectionVector, int& i, const char& Section);
//
// Читает и возвращает число с плавающей точкой начиная чтение с индекса i в разделах Global Section  и  Parameter Data 
// Section. Меняет значение индекса i на символ, следующий за прочитанным текстом. Аргумент  Section  показывает раздел, 
// в котором происходит чтение ('G'- Global Section, 'P'- Parameter Data Section).
double ReadRealDataType(const vector<string>& SectionVector, int& i, const char& Section);
//
// Читает и возвращает целое число в разделе Directory Entry для объекта на строке D_Sect_Str_Number в поле FieldNumber.
// Если поле заполнено пробелами - возвращает 0. Если в поле находится текст вместо целого числа - ошибка и выход из программы
int ReadIntegerIn_D_Section(const vector<string>& DSectionVector, const int D_Sect_Str_Number, const int FieldNumber);
//
// Читает цвет (т.е. расшифровывает поле 13 - Color Number) для любого Entity на строке Entity_D_Sect_Str_Number. 
// Возвращает: "стандарный" цвет (0-8), или нестандартный цвет - 10-значное число формата 1RGB (например для  
// RGB:254,001,220 это 1254001220). "Стандарные": 0-no color(default), 1-Black(RGB:0,0,0), 2-Red (RGB:255,0,0), 3-Green  
// (RGB:0,255,0), 4-Blue(RGB:0,0,255), 5-Yellow(RGB:255,255,0), 6-Magenta (RGB:255,0,255), 7-Cyan (RGB:0,255,255), 8-White (RGB:255,255,255).
int ReadEntityColor(const vector<string>& D_SectionVector, const vector<string>& P_SectionVector, const int D_Sect_Str_Number);
//
// Читает "Отрезок" (Line Entity - Type 110) в Directory Entry Section на строке D_Sect_Str_Number.
// Обрабатывает и записывает информацию в вектор Global_Entity110_Line_Vector. 
void ReadEntity110_Line(const vector<string>& D_SectionVector, const vector<string>& P_SectionVector, const int D_Sect_Str_Number);
// 
// Читает матрицу трансформации (Transformation Matrix Entity - Type 124) в Directory Entry Section, которая
// находится на строке Entity124_D_Sect_Str_Number, а также читает все входящие в неё матрицы (если такие есть). Записывает результат в TransformationMatrixVector.
void ReadEntity124_TrMatrix(vector<Entity124_TransformationMatrix>& TransformationMatrixVector, const vector<string>& D_SectionVector,
	const vector<string>& P_SectionVector, int Entity124_D_Sect_Str_Number);
//
// Преобразует координаты точки Xorigin, Yorigin, Zorigin (заданы в IGES-файле) в координаты X1, Y1, Z1 используя матрицы 
// преобразования  TransformationMatrixVector. Единицы измерения сохраняются.
void TransformPointByTransfMatrix(const double& Xorigin, const double& Yorigin, const double& Zorigin, double& X1, double& Y1, double& Z1, vector<Entity124_TransformationMatrix>& TransformationMatrixVector);
//
const int Global_MaxSignifDigits{ 15 };  // Максимальное количество значащих цифр, которое способно уместиться в double без потери точности
const int Global_GSectionLineLength{ 72 };  // Длина строки с информацией в Global Section
const int Global_PSectionLineLength{ 64 };  // Длина строки с информацией в Parameter Data Section
const int Global_D_Section_Field_Size{ 8 };  // Количество символов занимаемое одним полем в Directory Entry Section
const int Global_IGES_File_StringLength{ 80 };  // Длина текстовой строки в IGES-файле
const int Global_Entity110LineNumber = 110;  // Номер объекта "Отрезок" (Line Entity - Type 110)
const int Global_Entity314ColorNumber = 314;  // Номер объекта "Цвет" (Color Definition Entity - Type 314)
const int Global_Entity124TransMatrixNumber = 124;  // Номер объекта "Матрица трансформации" (Transformation Matrix Entity - Type 124)
g_GParameters Global_GlobalParameters;   // Глобальные параметры IGES-файла (из раздела Global Parameter)
vector<Entity110_Line> Global_Entity110_Line_Vector;  // Вектор структур Entity110_Line - Массив с информацией о всех Отрезках (объектах Entity Type 110 (Line))
// 
//
//
int main() {
	setlocale(LC_ALL, "Russian");
	const int IGESfileStringLength{ 80 };  // Длина строки данных в IGES-файле
	const int TmpTxtLength{ 82 };  //  Длина временной строки для чтения строк из IGES-файла
	const char IGESFileName[]{ "geom.igs" };  // Имя IGES-файла исходных данных 
	const char InfoFileName[]{ "info.txt" };  // Имя файла, куда в удобном виде записывается расшифрованная информация из IGES-файла
	const char Entity110_Line_FileName[]{ "Entity110_Line.txt" };   // Имя файла, куда будет записана информация об объектах Entity110 "Line" ("Отрезок")
	int Choice{ 0 };  // Выбор следующего действия
	char TmpTxt[TmpTxtLength];
	string InfoFileText;  // Текст для записи в файл "info.txt"
	string Entity110_Line_FileText;  // Текст для записи в файл "Entity110_Line.txt"
	FILE* IGESFilePointer{ nullptr }; // Указатель на открытый IGES-файл исходных данных "geom.igs"
	FILE* InfoFilePointer{ nullptr }; // Указатель на файл "info.txt" 
	FILE* Entity110_Line_FilePointer{ nullptr }; // Указатель на файл "Entity110_Line.txt" с информацией об объектах Entity110 "Line" ("Отрезок")
	//  
	vector<string>TmpInputVector;   //  Временный массив для первоначальной записи всего содержимого IGES-файла
	//  Массивы для записи строк из разделов IGES-файла
	vector<string> S_SectionVector;  // Массив для записи строк из первого раздела - Start Section
	vector<string> G_SectionVector;  // Массив для записи строк из второго раздела - Global Section
	vector<string> D_SectionVector;  // Массив для записи строк из третьего раздела - Directory Entry Section
	vector<string> P_SectionVector;  // Массив для записи строк из четвёртого раздела - Parameter Data Section
	vector<string> T_SectionVector;  // Массив для записи строки из последнего раздела - строки Terminate Section
	//
	//
	//  Печатаем Преамбулу и Главное меню
	while (true) {
		PrintPreamble(IGESFileName);  //  Печатаем Преамбулу
		//
		cout << "Введите цифру для дальнейшего действия:" << endl;
		SetColor(15);   cout << "0"; SetColor(7); cout << " - Продолжить выполнение программы; " << endl;
		cout << "1 - Справка по программе;" << endl;
		cout << "2 - Выход из программы." << endl;
		cout << " Ваш выбор: ";
		SetColor(13);     Choice = getInt_WithValueCheck(0, 2);     SetColor(7);
		if (Choice == 0) {  //  0 - Продолжить выполнение программы
			break;
		}
		else if (Choice == 1) {  //  1 - Справка по программе
			PrintHelp();  // Печатаем Справку по программе и создаём текстовый файл с текстом Справки
		}
		else if (Choice == 2) {  //  2 - Выход из программы
			exit(0);
		}
		system("cls");
	}
	//  Если выбран 0 - то продолжаем выполнение программы
	while (true) {  // Открываем IGES-файл. Выход из цикла только после успешного открытия файла
		system("cls");
		cout << "\n   Чтение данных из IGES-файла  " << IGESFileName << " :" << endl;
		_set_errno(0);
		if (fopen_s(&IGESFilePointer, IGESFileName, "r")) {  //  Если ошибка при открытии файла
			SetColor(12);  cout << "\n  Ошибка при открытии файла ";  SetColor(15);  cout << IGESFileName << endl;  SetColor(7);
			if (errno == ENOENT) {  // Если файл отсутствует
				SetColor(12);   cout << "  Файл с таким именем в текущей папке НЕ НАЙДЕН\n  (обратите внимание - все символы имени файла должны быть маленькими)." << endl;   SetColor(7);
			}
			cout << "\n\nВведите цифру для дальнейшего действия:" << endl;
			SetColor(15);   cout << "0"; SetColor(7); cout << " - Повторить чтение из файла  " << IGESFileName << " ;" << endl;
			cout << "1 - Выход из программы." << endl;
			cout << " Ваш выбор: ";
			SetColor(13);     Choice = getInt_WithValueCheck(0, 1);     SetColor(7);
			if (Choice == 0) {  // 0 - Повторить чтение из файла
				continue;
			}
			else if (Choice == 1) {   //  1 - Выход из программы
				exit(0);
			}
		}
		else  break;     //  Если файл открыт успешно - то выход их цикла и продолжение программы
	}
	SetColor(10);  cout << "\n   Файл ";  SetColor(15);  cout << IGESFileName;  SetColor(10); cout << " открыт успешно." << endl << endl;  SetColor(7);
	// Читаем содержимое IGES-файла во временный вектор строк  TmpInputVector
	while (fgets(TmpTxt, TmpTxtLength, IGESFilePointer) != NULL) { // Записали в TmpTxt строку, включая символ новой строки \n
		TmpTxt[IGESfileStringLength] = '\0';
		TmpInputVector.push_back(TmpTxt);
	}
	_fcloseall();
	// Копируем содержимое TmpInputVector в векторы по разделам
	{  // Вложенный блок - чтобы уничтожить в конце блока переменную  i
		int i = 0;  // индекс элемента в TmpInputVector (т.е. индекс обрабатываемой строки)
		// Копируем из TmpInputVector первый раздел (Start Section) в вектор S_SectionVector
		if (TmpInputVector[i][72] == 'S')
		{
			while (TmpInputVector[i][72] == 'S') {
				S_SectionVector.push_back(TmpInputVector[i]);
				i++;
			}
		}
		else { PrintErrorAndExit("   Ошибка! Не найден раздел Start Section"); }
		// Копируем из TmpInputVector второй раздел (Global Section) в вектор G_SectionVector
		if (TmpInputVector[i][72] == 'G')
		{
			while (TmpInputVector[i][72] == 'G') {
				G_SectionVector.push_back(TmpInputVector[i]);
				i++;
			}
		}
		else { PrintErrorAndExit("   Ошибка! Не найден раздел Global Section"); }
		// Копируем из TmpInputVector третий раздел (Directory Entry Section) в вектор D_SectionVector
		if (TmpInputVector[i][72] == 'D')
		{
			while (TmpInputVector[i][72] == 'D') {
				D_SectionVector.push_back(TmpInputVector[i]);
				i++;
			}
		}
		else { PrintErrorAndExit("   Ошибка! Не найден раздел Directory Entry Section"); }
		// Копируем из TmpInputVector четвёртый раздел (Parameter Data Section) в вектор P_SectionVector
		if (TmpInputVector[i][72] == 'P')
		{
			while (TmpInputVector[i][72] == 'P') {
				P_SectionVector.push_back(TmpInputVector[i]);
				i++;
			}
		}
		else { PrintErrorAndExit("   Ошибка! Не найден раздел Parameter Data Section"); }
		// Копируем из TmpInputVector последний раздел (строку Terminate Section) в вектор T_SectionVector
		if (TmpInputVector[i][72] == 'T') T_SectionVector.push_back(TmpInputVector[i]);
		else { PrintErrorAndExit("   Ошибка! Не найден раздел Terminate Section"); }
	}
	// Проверка загруженных данных
	{
		int S, G, D, P;  // Значения полей в разделе Terminate Section
		// Проверка раздела Directory Entry Section: количество строк должно быть чётным
		if (D_SectionVector.size() % 2 != 0)
			PrintErrorAndExit("   Ошибка! В разделе Directory Entry Section нечётное количество строк.");
		//  Читаем поле S раздела Terminate Section
		if (T_SectionVector[0][0] == 'S')   S = stoi(T_SectionVector[0].substr(1, 7));
		else {
			PrintErrorAndExit("   Ошибка! Не удалось прочитать поле S в разделе Terminate Section.");
		}
		//  Читаем поле G раздела Terminate Section
		if (T_SectionVector[0][8] == 'G')   G = stoi(T_SectionVector[0].substr(9, 7));
		else {
			PrintErrorAndExit("   Ошибка! Не удалось прочитать поле G в разделе Terminate Section.");
		}
		//  Читаем поле D раздела Terminate Section
		if (T_SectionVector[0][16] == 'D')   D = stoi(T_SectionVector[0].substr(17, 7));
		else {
			PrintErrorAndExit("   Ошибка! Не удалось прочитать поле D в разделе Terminate Section.");
		}
		//  Читаем поле P раздела Terminate Section
		if (T_SectionVector[0][24] == 'P')   P = stoi(T_SectionVector[0].substr(25, 7));
		else {
			PrintErrorAndExit("   Ошибка! Не удалось прочитать поле P в разделе Terminate Section.");
		}
	}
	//  Читаем в InfoFileText первый раздел (Start Section) из вектора S_SectionVector
	InfoFileText += "    Расшифрованные данные из файла  \"";
	InfoFileText += IGESFileName;
	InfoFileText += "\"\n    -----------------------------------------------\n\n";
	InfoFileText += "    Start Section:\n";
	InfoFileText += "    --------------\n";
	for (string& elem : S_SectionVector) {
		InfoFileText += elem.substr(0, 72);
		InfoFileText += '\n';
	}
	InfoFileText += '\n';
	// Читаем в InfoFileText Глобальные Параметры из второго раздела (Global Section) 
	InfoFileText = InfoFileText + "\n    Global Section:\n" + "    ----------------\n";  // Текст для дальнейшей записи в файл
	// Читаем раздел Global Section
	ReadAndCheckGlobalParameters(G_SectionVector, InfoFileText);
	// Читаем объекты из раздела Directory Entry Section
	{
		int D_Sect_Str_Number{ 1 }; // Номер строки очередного Entity в Directory Entry
		int EntityNumber{ 0 }; // Номер (Entity Type Number) элемента, который читается в настоящий момент
		while (D_Sect_Str_Number < D_SectionVector.size()) {
			EntityNumber = ReadIntegerIn_D_Section(D_SectionVector, D_Sect_Str_Number, 1);
			switch (EntityNumber)
			{
			case 100:  // 100 - Circular Arc
				break;
			case 104:  // 104 - Conic Arc
				break;
			case 110:  // 110 - Line
				ReadEntity110_Line(D_SectionVector, P_SectionVector, D_Sect_Str_Number);
				break;
			case 112:  // 112 - Parametric Spline Curve
				break;
			case 126:  // 126 - Rational B-Spline Curve 
				break;

			default:
				break;
			}
			D_Sect_Str_Number += 2;
		}

	}

	// Удалить
	cout << "\n____Информация о прочитанных объектах Entity110_Line____" << endl;
	for (Entity110_Line Entity110LineOBJ : Global_Entity110_Line_Vector) {
		cout << "Объект 110: строка №" << Entity110LineOBJ.D_Section_String_Number << " в Directory Entry,  строка №" << Entity110LineOBJ.P_Section_String_Number << " в Parameter Data." << endl;
		cout << "            Color = " << Entity110LineOBJ.Color << ".  FormNumber = " << Entity110LineOBJ.FormNumber << endl;
		if (Entity110LineOBJ.TransformationMatrixVector.empty()) continue;
		for (const Entity124_TransformationMatrix& TransMatrix : Entity110LineOBJ.TransformationMatrixVector) {
			cout << "       Transformation Matrix: " << endl;
			printf("       %.3f     %.3f     %.3f         %.3f\n", TransMatrix.R11, TransMatrix.R12, TransMatrix.R13, TransMatrix.T1);
			printf("       %.3f     %.3f     %.3f         %.3f\n", TransMatrix.R21, TransMatrix.R22, TransMatrix.R23, TransMatrix.T2);
			printf("       %.3f     %.3f     %.3f         %.3f\n", TransMatrix.R31, TransMatrix.R32, TransMatrix.R33, TransMatrix.T3);
			//cout << "             " << TransMatrix.R11 << "     " << TransMatrix.R12 << "     " << TransMatrix.R13 << "          " << TransMatrix.T1 << endl;
			//cout << "             " << TransMatrix.R21 << "     " << TransMatrix.R22 << "     " << TransMatrix.R23 << "          " << TransMatrix.T2 << endl;
			//cout << "             " << TransMatrix.R31 << "     " << TransMatrix.R32 << "     " << TransMatrix.R33 << "          " << TransMatrix.T3 << endl;
		}
	}
	// Удалить    */


	// Печатаем на экран основную информацию о прочитанной геометрии (масштаб, единицы измерения)
	cout << "\n      ОСНОВНАЯ ИНФОРМАЦИЯ О ПРОЧИТАННОЙ ГЕОМЕТРИИ" << endl;
	cout << "   -------------------------------------------------------" << endl;
	//cout << "Масштаб пространства модели:  ";
	printf("   Масштаб пространства модели:  %e\n", Global_GlobalParameters.GP13_ModelSpaceScale);
	cout << "   Единица измерения:  ";
	if (Global_GlobalParameters.GP14_UnitsFlag == 2) cout << "мм (миллиметры)" << endl;
	else if (Global_GlobalParameters.GP14_UnitsFlag == 6) cout << "м (метры)" << endl;
	cout << "   -------------------------------------------------------" << endl;
	cout << "\n\n\n";

	//  Запись информации из строки InfoFileText  в файл "info.txt" 
	cout << "   -------------------------------------------------------" << endl;
	cout << "   Далее программа создаст файл   "; SetColor(15);  cout << InfoFileName; SetColor(7);  cout << "  с расшифрованной " << endl;
	cout << "   информацией из IGES-файла  ";  SetColor(15);  cout << IGESFileName; SetColor(7);  cout << " .  Если файл ";
	SetColor(15); cout << InfoFileName << endl; SetColor(7); cout << "   уже существует, то он будет удалён, а вместо него " << endl;
	cout << "   будет создан новый файл." << endl << endl;
	cout << "      Нажмите ENTER для продолжения" << endl;
	PressENTER();
	_set_errno(0);
	if (fopen_s(&InfoFilePointer, InfoFileName, "w")) {  //  Если ошибка при создании и открытии файла
		_set_errno(0);
		PrintErrorAndExit(string("\n  Ошибка при создании файла  ") + string(InfoFileName));
	}
	else {   //  если файл успешно создан и открыт для записи, то
		if (fputs(InfoFileText.c_str(), InfoFilePointer) >= 0) {  //  Записываем текст в файл       
			SetColor(10);  cout << "\nФайл  "; SetColor(15);  cout << InfoFileName;  SetColor(10);  cout << "  успешно записан." << endl; SetColor(7);
			_fcloseall();
			cout << "\nНажмите ENTER для продолжения" << endl;
			PressENTER();
		}
		else {
			SetColor(12);   cout << "\n  Ошибка при записи в файл ";   SetColor(15);   cout << InfoFileName;  SetColor(7);
			_fcloseall();
			cout << "\nНажмите ENTER для продолжения" << endl;
			PressENTER();
		}
	}
	_fcloseall();

	// --- Запись информации в файл "Entity110_Line.txt" 
	cout << "\n\n   -------------------------------------------------------" << endl;
	cout << "   Далее программа создаст файл  "; SetColor(15);  cout << Entity110_Line_FileName; SetColor(7);  cout << "  с информацией  " << endl;
	cout << "   об объектах Entity110 \"Line\" (\"Отрезок\").  Если файл  "; SetColor(15); cout << Entity110_Line_FileName << endl; SetColor(7);
	cout << "   уже существует, то он будет удалён, а вместо него будет создан новый файл." << endl << endl;
	cout << "      Нажмите ENTER для продолжения" << endl;
	PressENTER();
	// Формируем текст об объектах Entity110 "Line" ("Отрезок") для записи в файл "Entity110_Line.txt" 
	Entity110_Line_FileText += "Entity_Type X1origin Y1origin Z1origin X2origin Y2origin Z2origin X1 Y1 Z1 X2 Y2 Z2 Length Color FormNumber\n";
	for (Entity110_Line Entity110LineOBJ : Global_Entity110_Line_Vector) {
		char X1originTXT[_CVTBUFSIZE], Y1originTXT[_CVTBUFSIZE], Z1originTXT[_CVTBUFSIZE], X2originTXT[_CVTBUFSIZE], Y2originTXT[_CVTBUFSIZE], Z2originTXT[_CVTBUFSIZE],
			X1_TXT[_CVTBUFSIZE], Y1_TXT[_CVTBUFSIZE], Z1_TXT[_CVTBUFSIZE], X2_TXT[_CVTBUFSIZE], Y2_TXT[_CVTBUFSIZE], Z2_TXT[_CVTBUFSIZE], LengthTXT[_CVTBUFSIZE];
		if (_gcvt_s(X1originTXT, _CVTBUFSIZE, Entity110LineOBJ.X1origin, Global_MaxSignifDigits) ||    // Конвертируем double в string. Если ошибка при конвертировании, то выход из программы
			_gcvt_s(Y1originTXT, _CVTBUFSIZE, Entity110LineOBJ.Y1origin, Global_MaxSignifDigits) ||
			_gcvt_s(Z1originTXT, _CVTBUFSIZE, Entity110LineOBJ.Z1origin, Global_MaxSignifDigits) ||
			_gcvt_s(X2originTXT, _CVTBUFSIZE, Entity110LineOBJ.X2origin, Global_MaxSignifDigits) ||
			_gcvt_s(Y2originTXT, _CVTBUFSIZE, Entity110LineOBJ.Y2origin, Global_MaxSignifDigits) ||
			_gcvt_s(Z2originTXT, _CVTBUFSIZE, Entity110LineOBJ.Z2origin, Global_MaxSignifDigits) ||
			_gcvt_s(X1_TXT, _CVTBUFSIZE, Entity110LineOBJ.X1, Global_MaxSignifDigits) ||
			_gcvt_s(Y1_TXT, _CVTBUFSIZE, Entity110LineOBJ.Y1, Global_MaxSignifDigits) ||
			_gcvt_s(Z1_TXT, _CVTBUFSIZE, Entity110LineOBJ.Z1, Global_MaxSignifDigits) ||
			_gcvt_s(X2_TXT, _CVTBUFSIZE, Entity110LineOBJ.X2, Global_MaxSignifDigits) ||
			_gcvt_s(Y2_TXT, _CVTBUFSIZE, Entity110LineOBJ.Y2, Global_MaxSignifDigits) ||
			_gcvt_s(Z2_TXT, _CVTBUFSIZE, Entity110LineOBJ.Z2, Global_MaxSignifDigits) ||
			_gcvt_s(LengthTXT, _CVTBUFSIZE, Entity110LineOBJ.Length, Global_MaxSignifDigits))
		{
			PrintErrorAndExit("Ошибка при вызове _gcvt_s() при формировании Entity110_Line_FileText.");
		}
		Entity110_Line_FileText = Entity110_Line_FileText + "Entity100_\"Line\" " + X1originTXT + " " + Y1originTXT + " " + Z1originTXT + " " +
			X2originTXT + " " + Y2originTXT + " " + Z2originTXT + " " + X1_TXT + " " + Y1_TXT + " " + Z1_TXT + " " +
			X2_TXT + " " + Y2_TXT + " " + Z2_TXT + " " + LengthTXT + " " + to_string(Entity110LineOBJ.Color) + " " + to_string(Entity110LineOBJ.FormNumber) + " \n";
	}
	// Запись в файл "Entity110_Line.txt"
	_set_errno(0);
	if (fopen_s(&Entity110_Line_FilePointer, Entity110_Line_FileName, "w")) {  //  Если ошибка при создании и открытии файла
		_set_errno(0);
		PrintErrorAndExit(string("\n  Ошибка при создании файла  ") + string(Entity110_Line_FileName));
	}
	else {   //  если файл успешно создан и открыт для записи, то
		if (fputs(Entity110_Line_FileText.c_str(), Entity110_Line_FilePointer) >= 0) {  //  Записываем текст в файл       
			SetColor(10);  cout << "\nФайл  "; SetColor(15);  cout << Entity110_Line_FileName;  SetColor(10);  cout << "  успешно записан." << endl; SetColor(7);
			_fcloseall();
			cout << "\nНажмите ENTER для продолжения" << endl;
			PressENTER();
		}
		else {
			SetColor(12);   cout << "\n  Ошибка при записи в файл ";   SetColor(15);   cout << Entity110_Line_FileName;  SetColor(7);
			_fcloseall();
			cout << "\nНажмите ENTER для продолжения" << endl;
			PressENTER();
		}
	}
	_fcloseall();
	// --- Определение матрицы трансформации для пересчёта из координат 3D-модели в судовую систему координат ---
	{
		// Поиск в Global_Entity110_Line_Vector отрезков осей - 3 самых длинных отрезка 
		int AxisX_index{ 0 }, AxisY_index{ 0 }, AxisZ_index{ 0 };  // Индекс положения отрезков осей в векторе Global_Entity110_Line_Vector. AxisX_index - индекс самого длинного отрезка. AxisY_index - индекс второго по длине отрезка. AxisZ_index - индекс третьего по длине отрезка
		for (int i = 0; i < Global_Entity110_Line_Vector.size(); i++) {
			if (Global_Entity110_Line_Vector[i].Length > Global_Entity110_Line_Vector[AxisX_index].Length) 	AxisX_index = i;
			else if (Global_Entity110_Line_Vector[i].Length > Global_Entity110_Line_Vector[AxisY_index].Length)	AxisY_index = i;
			else if (Global_Entity110_Line_Vector[i].Length > Global_Entity110_Line_Vector[AxisZ_index].Length)	AxisZ_index = i;
		}
		cout << "\n\nAxisX_index=" << AxisX_index << ";   AxisY_index=" << AxisY_index << ";   AxisZ_index=" << AxisZ_index << endl << endl;  //  Удалить
		// Проверка найденных отрезков осей. Они должны быть в одном экземпляре, параллельны осям координат 3D-модели и взаимно перпендикулярны друг другу.
		for (int i = 0; i < Global_Entity110_Line_Vector.size(); i++) {   //  Проверка, что отрезок "ось X" (самый длинный отрезок) - в единственном экземпляре
			if (i == AxisX_index) continue;
			if (Global_Entity110_Line_Vector[i].Length == (Global_Entity110_Line_Vector[AxisX_index].Length)) { // Если найден второй отрезок "ось X" - ошибка и выход
				string txt = "Ошибка в исходном файле!\nНайден второй отрезок \'ось X\' (самый длинный отрезок в 3D-модели).\n";
				txt = txt + "Отрезок \'ось X\' должен быть только один.\n  Информация о найденных отрезках:\n" +
					"Oтрезок 1: X1=" + to_string(Global_Entity110_Line_Vector[AxisX_index].X1) + "; Y1=" + to_string(Global_Entity110_Line_Vector[AxisX_index].Y1) +
					"; Z1=" + to_string(Global_Entity110_Line_Vector[AxisX_index].Z1) + ";      X2=" + to_string(Global_Entity110_Line_Vector[AxisX_index].X2) +
					"; Y2=" + to_string(Global_Entity110_Line_Vector[AxisX_index].Y2) + "; Z2=" + to_string(Global_Entity110_Line_Vector[AxisX_index].Z2) +
					".\n Oтрезок 2: X1=" + to_string(Global_Entity110_Line_Vector[i].X1) + "; Y1=" + to_string(Global_Entity110_Line_Vector[i].Y1) +
					"; Z1=" + to_string(Global_Entity110_Line_Vector[i].Z1) + ";      X2=" + to_string(Global_Entity110_Line_Vector[i].X2) +
					"; Y2=" + to_string(Global_Entity110_Line_Vector[i].Y2) + "; Z2=" + to_string(Global_Entity110_Line_Vector[i].Z2) + ".\n";
				PrintErrorAndExit(txt);
			}
		}

	}




	// Завершение программы
	SetColor(10);    cout << "       Программа завершена успешно." << endl << endl;    SetColor(7);
	cout << "    Нажмите ENTER для выхода из программы" << endl;
	PressENTER();
}   //   Конец функции main()
//
// 
//  Печатает преамбулу 
void PrintPreamble(const char* SourceFileName)
{
	system("cls");
	cout << "  ---------------------------------------------------------------------------  " << endl;
	cout << "  ---------------    Программа   IGES-SeaHydro_Converter    -----------------  " << endl;
	cout << "  ---------------                 Версия 1.0.               -----------------  " << endl;
	cout << "  ---------------------------------------------------------------------------  " << endl;
	cout << endl;
	cout << "  Программа  \"IGES-SeaHydro_Converter\" читает 3D-геометрию формы" << endl;
	cout << "  корпуса судна из файла типа IGES (расширение.igs) и создаёт новый файл" << endl;
	cout << "  с исходными данными для программы  \"SeaHydro\"." << endl;
	cout << endl;
	cout << "  Программа читает информацию о следующих типах линий из IGES-файла:" << endl;
	SetColor(12); cout << "  - \"Circular Arc\" (Type Number 100) (_НЕТ_В_ДАННОЙ_ВЕРСИИ_);" << endl;  SetColor(7);
	SetColor(12); cout << "  - \"Conic Arc\" (Type Number 104) (_НЕТ_В_ДАННОЙ_ВЕРСИИ_);" << endl;  SetColor(7);
	SetColor(10);  cout << "  - \"Line\" (Type Number 110);" << endl;  SetColor(7);
	SetColor(12); cout << "  - \"Parametric Spline Curve\" (Type Number 112) (_НЕТ_В_ДАННОЙ_ВЕРСИИ_);" << endl;  SetColor(7);
	SetColor(12); cout << "  - \"Rational B-Spline Curve\" (Type Number 126) (_НЕТ_В_ДАННОЙ_ВЕРСИИ_);" << endl;  SetColor(7);
	cout << endl;
	cout << "Требования к IGES-файлу указаны в Справке." << endl;
	cout << endl;
}
//
//
// Печатает Справку по программе, создаёт текстовый файл help.txt с текстом справки
void PrintHelp(const char* HelpFileName)
{
	int Choice{ 0 };  // Выбор следующего действия
	FILE* HelpFilePointer{ nullptr }; // Указатель на файл 
	string heading = { "  ---------------------------------------------------------------------------  \n\
           Справка по программе   \"IGES-SeaHydro_Converter\"  версии 1.0.    \n\
  ---------------------------------------------------------------------------  \n"
	};
	// Текст Справки
	char txt[] = { "\
1. Описание программы.                                                        \n\
  Программа IGES-SeaHydro_Converter.exe читает IGES-файл (расширение .igs)    \n\
  c 3D-геометрией формы корпуса судна и создаёт новый файл с координатами     \n\
  шпангоутов судна для использования в расчётах остойчивости и непотопляемости\n\
  в программе \"SeaHydro\" (разработчик программы \"SeaHydro\"- фирма ООО \"Си Тех\",\n\
  сайт www.seatech.ru).                                                       \n\
\n\
  Программа читает следующие типы линий (curve entities) из IGES-файла:       \n\
         - \"Circular Arc\" (Type Number 100) (_НЕТ_В_ДАННОЙ_ВЕРСИИ_);        \n\
         - \"Conic Arc\" (Type Number 104) (_НЕТ_В_ДАННОЙ_ВЕРСИИ_);           \n\
  - \"Line\" (Type Number 110);                                               \n\
         - \"Parametric Spline Curve\" (Type Number 112) (_НЕТ_В_ДАННОЙ_ВЕРСИИ_);\n\
         - \"Rational B-Spline Curve\" (Type Number 126) (_НЕТ_В_ДАННОЙ_ВЕРСИИ_).\n\
  Другие объекты в IGES-файле (поверхности, тела, точки и др.) игнорируются и \n\
  не влияют на расчёт.                                                        \n\
\n\
2. Требования к исходному IGES-файлу 3D-модели.                               \n\
 2.1 Общие требования к файлу:                                                \n\
   - имя файла - \"geom.igs\" (все буквы маленькие). Если IGES-файл изначально\n\
     имеет другое имя - то переименуйте его в \"geom.igs\";                   \n\
   - файл \"geom.igs\" должен быть в одной папке с exe-файлом программы.      \n\
 2.2 Требования к геометрии:                                                  \n\
  2.2.1 Общие требования к геометрии:                                         \n\
   - единицы измерения в геометрии - миллиметры (ММ) или метры (М).           \n\
   - масштаб пространства модели = 1.0;                                       \n\
   - свойства линий (цвет, толщина, слой, тип линии) не имеют значения;       \n\
   - координаты линий будут округлены с точностью 0,1 мм.                     \n\
  2.2.2 Линии, читаемые программой, делятся на три группы:                    \n\
   - отрезки осей X, Y, Z - три взаимно перпендикулярных отрезка (объекты Type\n\
     Number 110 \"Line\"), исходящих из одной точки. Определяют судовую систему\n\
     координат независимо от системы координат 3D-модели в IGES-файле;        \n\
   - отрезки шпаций - непрерывная цепочка отрезков, параллельно оси X в       \n\
     плоскости ДП ниже оси X. Через середину каждого отрезка проходит плоскость\n\
     шпангоута. Длина отрезка равна длине расчётной шпации данного шпангоута; \n\
   - линии шпангоутов - отрезки, дуги, сплайны и т.п., указанные в п.1 Справки,\n\
     лежащие в плоскостях шпангоутов. Описывают геометрию каждого шпангоута.  \n\
  2.2.3 Требования к отрезкам осей:                                           \n\
   - отрезки осей должны быть объектами типа \"Line\" (Entity Type Number 110);\n\
   - отрезки осей должны быть самыми длинными отрезками в 3D-модели. Отрезок  \n\
     \'ось X\' - самый длинным отрезок, отрезок \'ось Y\' короче чем X, отрезок\n\
     \'ось Z\' короче чем Y. Все остальные отрезки в 3D-модели должны быть короче;\n\
   - отрезки осей должны начинаться в одной точке, быть взаимно перпендикулярны,\n\
     каждый отрезок должен быть параллелен какой-либо оси координат 3D-модели.\n\
     Точка пересечения отрезков осей будет являться началом кооординат судовой\n\
     системы координат результирующего файла;                                 \n\
   - отрезки осей образуют левую систему координат: отрезок \'ось X\' направлен\n\
     в нос судна, отрезок \'ось Y\' - на правый борт, отрезок \'ось Z\' - вверх.\n\
     Отрезки X-Y определяют основную плоскость (ОП), отрезки X-Z определяют   \n\
     диаметральную плоскость (ДП), отрезки Y-Z - плоскость мидель-шпангоута.  \n\
  2.2.4 Требования к отрезкам шпаций:                                         \n\
   - отрезки шпаций должны быть объектами типа \"Line\" (Entity Type Number 110);\n\
   - отрезки шпаций должны лежать цепочкой без зазоров в плоскости ДП         \n\
     параллельно оси X ниже любых других отрезков в файле.                    \n\
   - длина отрезка шпации равна длине расчётной шпации данного шпангоута.     \n\
  2.2.5 Требования к линиям шпангоутов:                                       \n\
   - контур каждого шпангоута должен быть замкнутой цепочкой линий шпангоутов \n\
     и лежать в плоскости, проходящей через середину отрезка шпации.          \n\
   - в данной версии программы (Версия 1.0.) линии шпангоутов должны быть     \n\
     объектами типа:                                                          \n\
      - \"Line\" (Type Number 110).                                           \n\
                                                                              \n\
3. Материалы про IGES-файлы:                                                  \n\
  - Статья \"Структура IGES-файла\":                                          \n\
       (сайт  https://unril.wordpress.com/2011/04/01/structure-iges-file/ );  \n\
  - \"The Initial Graphics Exchange Specification(IGES) Version 6.0\"           \n\
       (сайт  https://www.filemonger.com/specs/igs/devdept.com/version6.pdf );\n\
  - книга  Кунву Ли \"Основы САПР (CAD/CAM/CAE)\".-СПб.: Питер, 2004 .          \n\
"
	};
	system("cls");
	cout << endl;
	cout << heading; // Печать заголовка справки
	cout << "Введите цифру для дальнейшего действия:" << endl;
	cout << "0 - Показать Справку на экране;" << endl;
	cout << "1 - Создать файл "; SetColor(15); cout << HelpFileName; SetColor(7); cout << " с текстом Справки; " << endl;
	cout << "2 - Выход из Справки." << endl;
	cout << " Ваш выбор: ";
	SetColor(13);     Choice = getInt_WithValueCheck(0, 2);     SetColor(7);
	// 
	if (Choice == 0) {  // 0 - Напечатать Справку на экране
		system("cls");
		cout << heading; // Печать заголовка справки
		cout << txt;
		cout << "\nНажмите ENTER для выхода из Справки" << endl;
		while (true) {
			if (_getch() == 13) break;
		}
	}
	else if (Choice == 1) {   // 1 - Создать файл
		cout << endl << endl;
		cout << "-------------------------------------------------------------------------------" << endl;
		cout << "Программа создаст файл "; SetColor(15); cout << HelpFileName; SetColor(7); cout << ". Если файл с таким именем уже существует, то он" << endl;
		cout << "будет удалён, а вместо него будет создан новый файл." << endl;
		cout << "Введите цифру для дальнейшего действия:" << endl;
		SetColor(15);  cout << "0";  SetColor(7);  cout << " - Создать файл Справки \"";  cout << HelpFileName;  cout << "\" ." << endl;
		cout << "1 - Выйти из Справки." << endl;
		cout << " Ваш выбор: ";
		SetColor(13);     Choice = getInt_WithValueCheck(0, 1);     SetColor(7);
		if (Choice == 0) {   //  0 - Создать файл Справки
			_set_errno(0);
			if (fopen_s(&HelpFilePointer, HelpFileName, "w")) {  //  Если ошибка при создании и открытии файла
				_set_errno(0);
				SetColor(12);  cout << "\n  Ошибка при создании файла ";  SetColor(15);  cout << HelpFileName;  SetColor(7);
				cout << "\nНажмите ENTER для продолжения" << endl;
				PressENTER();
			}
			else {   //  если файл успешно создан и открыт для записи, то
				heading += txt;
				if (fputs(heading.c_str(), HelpFilePointer) >= 0) {  //  Записываем текст в файл       
					SetColor(10);  cout << "\nФайл Справки  "; SetColor(15);  cout << HelpFileName;  SetColor(10);  cout << "  успешно записан." << endl; SetColor(7);
					_fcloseall();
					cout << "\nНажмите ENTER для продолжения" << endl;
					PressENTER();
				}
				else {
					SetColor(12);   cout << "\n  Ошибка при записи в файл ";   SetColor(15);   cout << HelpFileName;  SetColor(7);
					_fcloseall();
					cout << "\nНажмите ENTER для продолжения" << endl;
					PressENTER();
				}
			}
			_fcloseall();
		}
		else if (Choice == 1) {
			system("cls");
			return;
		}
	}
	cout << endl << endl;
}
//
// Изменение цвета печатаемого текста и фона. Black = 0, Blue = 1, Green = 2, Cyan = 3, Red = 4, Magenta = 5, Brown = 6, LightGray = 7,
//  DarkGray = 8, LightBlue = 9, LightGreen = 10, LightCyan = 11, LightRed = 12, LightMagenta = 13, Yellow = 14, White = 15. 
//  Стандарный цвет текста - SetColor(7,0).
void SetColor(int TextColor, int bg)
{
	HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hStdOut, (WORD)((bg << 4) | TextColor));
}
//
// Ввод числа типа int с проверкой
int getInt()
{
	while (true) // цикл продолжается до тех пор, пока пользователь не введёт корректное значение
	{
		int a;
		cin >> a;
		// Проверка на предыдущее извлечение
		if (cin.fail()) // если предыдущее извлечение оказалось неудачным,
		{
			cin.clear(); // то возвращаем cin в 'обычный' режим работы
			cin.ignore(LLONG_MAX, '\n');    // и удаляем значения предыдущего ввода из входного буфера   
			cout << "Ошибка!!! Должно быть целое число. Попробуйте ещё раз." << endl;
		}
		else
		{
			cin.ignore(LLONG_MAX, '\n'); // удаляем лишние значения
			return a;
		}
	}
}
//
// Ввод числа типа int с проверкой: возвращаются значения только от Min до Max включительно.
int getInt_WithValueCheck(int Min, int Max)
{
	int tmp;
	if (Min > Max) {
		tmp = Min;
		Min = Max;
		Max = tmp;
	}
	while (true) {
		tmp = getInt();
		if (tmp >= Min && tmp <= Max) break;
		cout << "Ошибка!!! Число должно быть от " << Min << " до " << Max << ". Попробуйте ещё раз" << endl;
	}
	return tmp;
}
//
// Ожидает нажатия клавиши ENTER. Все остальные нажатия игнорируются, текст не печатается
void PressENTER() {
	while (true) {
		if (_getch() == 13) break;
	}
}
// 
//  Печатает красным текстом сообщение и после нажатия ENTER выходит из программы 
void PrintErrorAndExit(const string& Text)
{
	SetColor(12);   cout << Text << endl << endl;   SetColor(7);
	cout << "   Нажмите ENTER  (выход из программы)" << endl;
	PressENTER();
	exit(0);
}
//
// Проверяет аргумент на соответствие требованиям к символам для Global Parameter 1 и Global Parameter 2 (по IGES Specification Version 6.0, 2.2.3.1).
// Возвращает true  если символ соответствует, иначе- false.
bool CheckGP1GP2(const char& ch)
{
	if ((ch >= 0 && ch <= 32) || ch == 127 || (ch >= 48 && ch <= 57) || ch == 43 || ch == 45 || ch == 46 || ch == 68 || ch == 69 || ch == 72) return false;
	else return true;
}
//
// Читает в структуру g_GParameters Глобальные Параметры из G_SectionVector, проверяет их по IGES Specification V.6.0. 
// Добавляет информацию о глобальных параметрах в InfoFileText. 
void ReadAndCheckGlobalParameters(const vector<string>& G_SectionVector, string& InfoFileText)
{
	// Читаем Parameter Delimiter и Record Delimiter  в соответствии с п.2.2.3.1  "The Initial Graphics Exchange Specification(IGES) Version 6.0"
	int i = 0;  // индекс символа в строке  G_SectionVector
	// Читаем Global Parameter 1 (Parameter Delimiter) 
	if (G_SectionVector[0][0] == ',') 	Global_GlobalParameters.GP1_ParameterDelimiter = ',';
	else if (G_SectionVector[0][0] == '1' && G_SectionVector[0][1] == 'H') {
		Global_GlobalParameters.GP1_ParameterDelimiter = G_SectionVector[0][2];
		i += 3;
	}
	else PrintErrorAndExit("\n  Ошибка!  Не найден Global Parameter 1 (Parameter Delimiter).");
	// Проверяем на корректность значение Global Parameter 1 (Parameter Delimiter) 
	if (!CheckGP1GP2(Global_GlobalParameters.GP1_ParameterDelimiter))
		PrintErrorAndExit("Ошибка!  Неверное значение Global Parameter 1 (Parameter Delimiter)");
	// Проверяем разделитель Parameter Delimiter после Global Parameter 1
	if (G_SectionVector[0][i] != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение  Parameter Delimiter  после  Global Parameter 1.");
	i++;
	//  Читаем Global Parameter 2 (Record Delimiter)
	if (G_SectionVector[0][i] == Global_GlobalParameters.GP1_ParameterDelimiter)  Global_GlobalParameters.GP2_RecordDelimiter = ';';  // Значение по умолчанию
	else if (G_SectionVector[0][i] == '1' && G_SectionVector[0][i + 1] == 'H') {
		Global_GlobalParameters.GP2_RecordDelimiter = G_SectionVector[0][i + 2];
		i += 3;
	}
	// Проверяем на корректность значение Global Parameter 2 (Record Delimiter)
	if (!CheckGP1GP2(Global_GlobalParameters.GP2_RecordDelimiter))
		PrintErrorAndExit("Ошибка!  Неверное значение Global Parameter 2 (Record Delimiter)");
	// Читаем разделитель после Global Parameter 2 
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 2"); // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем  Global Parameter 3 (Product identification)
	Global_GlobalParameters.GP3_ProductID = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// Читаем разделитель после  Global Parameter 3
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 3"); // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем  Global Parameter 4 (File Name)
	Global_GlobalParameters.GP4_FileName = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// Читаем разделитель после  Global Parameter 4
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 4");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем  Global Parameter 5 (Native System ID)
	Global_GlobalParameters.GP5_NativeSystemID = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// Читаем разделитель после  Global Parameter 5
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 5");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем  Global Parameter 6 (Preprocessor version). 
	Global_GlobalParameters.GP6_PreprocessorVersion = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// Читаем разделитель после  Global Parameter 6
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 6");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 7 (Number of Binary Bits for Integer Representation)
	Global_GlobalParameters.GP7_BitsForInteger = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// Читаем разделитель после  Global Parameter 7
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 7");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 8 (Single-Precision Magnitud - Maximum power of ten representable in a single-precision floating point number)
	Global_GlobalParameters.GP8_SinglePrecisionMagnitud = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// Читаем разделитель после  Global Parameter 8
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 8");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 9 (Single-Precision Significance - Number of significant digits in a single-precision floating point number)
	Global_GlobalParameters.GP9_SinglePrecisionSignificance = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// Читаем разделитель после  Global Parameter 9
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 9");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 10 (Double-Precision Magnitude - Maximum power of ten representable in a double-precision floating point number)
	Global_GlobalParameters.GP10_DoublePrecisionMagnitude = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// Читаем разделитель после  Global Parameter 10
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 10");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 11 (Double-Precision Significance - Number of significant digits in a double-precision floating point number)
	Global_GlobalParameters.GP11_DoublePrecisionSignificance = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// Читаем разделитель после  Global Parameter 11
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 11");   // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 12 (Product Identification for the Receiver. The default value is the value specified in parameter 3)
	if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 12");  // Если это Record Delimiter, то печать ошибки и выход
	else if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP1_ParameterDelimiter)
		Global_GlobalParameters.GP12_ProductIdForReceiver = Global_GlobalParameters.GP3_ProductID;   // Если это Parameter Delimiter, то присваиваем значение по умолчанию
	else 	// иначе - читаем значение
		Global_GlobalParameters.GP12_ProductIdForReceiver = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// Читаем разделитель после  Global Parameter 12
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 12");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 13 (Model Space Scale. The default value is 1.0)
	if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 13");  // Если это Record Delimiter, то печать ошибки и выход
	else if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP1_ParameterDelimiter)
		Global_GlobalParameters.GP13_ModelSpaceScale = 1.0;   // Если это Parameter Delimiter, то присваиваем значение по умолчанию
	else   // иначе - читаем значение
		Global_GlobalParameters.GP13_ModelSpaceScale = ReadRealDataType(G_SectionVector, i, 'G');
	// Читаем разделитель после  Global Parameter 13
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 13");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 14 (Units Flag - Код единицы измерения). default = 1 (inches);  2 - millimeters;  6 - meters.
	if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 14");  // Если это Record Delimiter, то печать ошибки и выход
	else if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP1_ParameterDelimiter)  // Если это Parameter Delimiter,
		Global_GlobalParameters.GP14_UnitsFlag = 1;                                          //  то присваиваем значение по умолчанию
	else   // иначе - читаем значение
		Global_GlobalParameters.GP14_UnitsFlag = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// Читаем разделитель после  Global Parameter 14
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 14");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 15 (Units Name)
	if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 15");  // Если это Record Delimiter, то печать ошибки и выход
	else if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP1_ParameterDelimiter)
		Global_GlobalParameters.GP15_UnitsName = "INCH";   // Если это Parameter Delimiter, то присваиваем значение по умолчанию
	else   // иначе - читаем значение
		Global_GlobalParameters.GP15_UnitsName = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// Читаем разделитель после  Global Parameter 15
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 15");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 16 (Maximum Number of Line Weight Gradations).
	if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 16");  // Если это Record Delimiter, то печать ошибки и выход
	else if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP1_ParameterDelimiter)
		Global_GlobalParameters.GP16_MaxNumLineWeightGradations = 1;   // Если это Parameter Delimiter, то присваиваем значение по умолчанию
	else   // иначе - читаем значение
		Global_GlobalParameters.GP16_MaxNumLineWeightGradations = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// Читаем разделитель после  Global Parameter 16
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 16");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 17 (Width of Maximum Line Weight in Units)
	Global_GlobalParameters.GP17_MaximumLineWidth = ReadRealDataType(G_SectionVector, i, 'G');
	// Читаем разделитель после  Global Parameter 17
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 17");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем  Global Parameter 18 (Date and Time of Exchange File Generation)
	Global_GlobalParameters.GP18_DateTimeFileGeneration = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// Читаем разделитель после  Global Parameter 18
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 18");  // Если это не Parameter Delimiter, то печать ошибки и выход 
	// Читаем Global Parameter 19 (Minimum User-Intended Resolution)
	Global_GlobalParameters.GP19_MinimumResolution = ReadRealDataType(G_SectionVector, i, 'G');
	// Поскольку все следующие Глобальные Параметры имеют значения по умолчанию, то присваиваем им значения по умолчанию
	Global_GlobalParameters.GP20_MaxCoordValue = 0.0;
	Global_GlobalParameters.GP21_AuthorName = "";
	Global_GlobalParameters.GP22_AuthorsOrganization = "";
	Global_GlobalParameters.GP23_VersionFlag = 3;
	Global_GlobalParameters.GP24_DraftingStandardFlag = 0;
	Global_GlobalParameters.GP25_DateTimeNativModelModified = "";
	Global_GlobalParameters.GP26_AppProtocolIdentifier = "";
	// Блок для чтения Глобальных Параметров с 20-го до 26-й. Выполняется 1 раз. Чтение прекращается после Record Delimiter
	while (true) {
		char TmpDelimiter;  // Временный символ для анализа прочитанного разделителя
		// Читаем разделитель после  Global Parameter 19
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter
			PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 19");  // то печать ошибки и выход. 
		// Читаем Global Parameter 20 (Approximate Maximum Coordinate Value)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter, то читаем значение
			Global_GlobalParameters.GP20_MaxCoordValue = ReadRealDataType(G_SectionVector, i, 'G');
		// Читаем разделитель после  Global Parameter 20
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter
			PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 20");  // то печать ошибки и выход. 
		// Читаем Global Parameter 21 (Name of Author)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter, то читаем значение
			Global_GlobalParameters.GP21_AuthorName = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
		// Читаем разделитель после  Global Parameter 21
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter
			PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 21");  // то печать ошибки и выход. 
		// Читаем Global Parameter 22 (Author’s Organization)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter, то читаем значение
			Global_GlobalParameters.GP22_AuthorsOrganization = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
		// Читаем разделитель после  Global Parameter 22
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter
			PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 22");  // то печать ошибки и выход. 
		// Читаем Global Parameter 23 (Version Flag)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter, то читаем значение
			Global_GlobalParameters.GP23_VersionFlag = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
		// Читаем разделитель после  Global Parameter 23
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter
			PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 23");  // то печать ошибки и выход. 
		// Читаем Global Parameter 24 (Drafting Standard Flag)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter, то читаем значение
			Global_GlobalParameters.GP24_DraftingStandardFlag = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
		// Читаем разделитель после  Global Parameter 24
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter
			PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 24");  // то печать ошибки и выход. 
		// Читаем Global Parameter 25 (Date and Time Model was Created or Modified)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter, то читаем значение
			Global_GlobalParameters.GP25_DateTimeNativModelModified = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
		// Читаем разделитель после  Global Parameter 25
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter
			PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 25");  // то печать ошибки и выход. 
		// Читаем Global Parameter 26 (Application Protocol/Subset Identifier)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // Прекращение чтения если Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // Если не Parameter Delimiter, то читаем значение
			Global_GlobalParameters.GP26_AppProtocolIdentifier = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
		// Читаем разделитель после  Global Parameter 26
		if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP2_RecordDelimiter) // Если не Record Delimiter
			PrintErrorAndExit("Ошибка!  Неверное значение разделителя после  Global Parameter 26");  // то печать ошибки и выход. 
		break;
	}
	// --- ПРОВЕРЯЕМ прочитанные ГЛОБАЛЬНЫЕ ПАРАМЕТРЫ ---
	// Проверяем Global Parameter 13 (Model Space Scale - Масштаб пространства модели). Должен быть 1.0, другие значения не обрабатываем - ошибка и выход из программы 
	if (Global_GlobalParameters.GP13_ModelSpaceScale != 1.) {
		char buffer[_CVTBUFSIZE];
		if (_gcvt_s(buffer, _CVTBUFSIZE, Global_GlobalParameters.GP13_ModelSpaceScale, Global_MaxSignifDigits)) { // Если ошибка при конвертировании double в string, то выходим из программы
			PrintErrorAndExit("  Ошибка при проверке значения Global Parameter 13 (Model Space Scale). \n\
				 Ошибка при конвертировании double в string.");
		}
		string txt{ "" };
		txt = txt + "Ошибка! Model Space Scale (Масштаб пространства модели) в IGES-файле НЕ равен 1.0. \n" +
			"Глобальный Параметр 13 (Model Space Scale) = " + buffer +
			".\nЗначение должно быть равно 1.0.";
		PrintErrorAndExit(txt);
	}
	// 
	// Проверяем Global Parameter 14 (Units Flag - Код единицы измерения). Должен быть 2 (millimeters) или 6 (meters). Иначе - ошибка и выход из программы
	if (Global_GlobalParameters.GP14_UnitsFlag != 2 && Global_GlobalParameters.GP14_UnitsFlag != 6) {
		string txt{ "" };
		txt = txt + "Ошибка! Единицы измерения в IGES-файле НЕ метры и НЕ миллиметры. \n" +
			"Глобальный Параметр 14 (Units Flag - Код единицы измерения) = " + to_string(Global_GlobalParameters.GP14_UnitsFlag) +
			".\nЗначение должно быть либо \'2\' (миллиметры), либо \'6\' (метры). ";
		PrintErrorAndExit(txt);
	}
	//
	// ДОБАВЛЯЕМ ПРОЧИТАННЫУЮ ИНФОРМАЦИЮ В СТРОКУ ДЛЯ ДАЛЬНЕЙШЕЙ ЗАПИСИ В ФАЙЛ
	// Записываем в InfoFileText значение Global Parameter 1 (Parameter Delimiter) 
	InfoFileText = InfoFileText + "Глобальный параметр 1 (Parameter Delimiter - Разделитель параметров) = " + Global_GlobalParameters.GP1_ParameterDelimiter + '\n';
	// Записываем в InfoFileText значение Global Parameter 2 (Record Delimiter)
	InfoFileText = InfoFileText + "Глобальный параметр 2 (Record Delimiter - Разделитель записей) = " + Global_GlobalParameters.GP2_RecordDelimiter + '\n';
	// Записываем в InfoFileText значение Global Parameter 3 (Product identification)
	InfoFileText = InfoFileText + "Глобальный параметр 3 (Product identification from sending system) = " + Global_GlobalParameters.GP3_ProductID + '\n';
	// Записываем в InfoFileText значение Global Parameter 4 (File Name)
	InfoFileText = InfoFileText + "Глобальный параметр 4 (File Name) = " + Global_GlobalParameters.GP4_FileName + '\n';
	// Записываем в InfoFileText значение Global Parameter 5 (Native System ID)
	InfoFileText = InfoFileText + "Глобальный параметр 5 (Native System ID) = " + Global_GlobalParameters.GP5_NativeSystemID + '\n';
	// Записываем в InfoFileText значение Global Parameter 6 (Preprocessor version)
	InfoFileText = InfoFileText + "Глобальный параметр 6 (Preprocessor version) = " + Global_GlobalParameters.GP6_PreprocessorVersion + '\n';
	// Записываем в InfoFileText значение Global Parameter 7 (Number of Binary Bits for Integer Representation)
	InfoFileText = InfoFileText + "Глобальный параметр 7 (Number of Binary Bits for Integer Representation) = " + to_string(Global_GlobalParameters.GP7_BitsForInteger) + '\n';
	// Записываем в InfoFileText значение Global Parameter 8 (Single-Precision Magnitud )
	InfoFileText = InfoFileText + "Глобальный параметр 8 (Single-Precision Magnitud - Maximum power of ten representable in a single-precision floating point number) = " + to_string(Global_GlobalParameters.GP8_SinglePrecisionMagnitud) + '\n';
	// Записываем в InfoFileText значение Global Parameter 9 (Single-Precision Significance)
	InfoFileText = InfoFileText + "Глобальный параметр 9 (Single-Precision Significance - Number of significant digits in a single-precision floating point number) = " + to_string(Global_GlobalParameters.GP9_SinglePrecisionSignificance) + '\n';
	// Записываем в InfoFileText значение Global Parameter 10 (Double-Precision Magnitude)
	InfoFileText = InfoFileText + "Глобальный параметр 10 (Double-Precision Magnitude - Maximum power of ten representable in a double-precision floating point number) = " + to_string(Global_GlobalParameters.GP10_DoublePrecisionMagnitude) + '\n';
	// Записываем в InfoFileText значение Global Parameter 11 Double-Precision Significance)
	InfoFileText = InfoFileText + "Глобальный параметр 11 (Double-Precision Significance - Number of significant digits in a double-precision floating point number) = " + to_string(Global_GlobalParameters.GP11_DoublePrecisionSignificance) + '\n';
	// Записываем в InfoFileText значение Global Parameter 12 (Product Identification for the Receiver)
	InfoFileText = InfoFileText + "Глобальный параметр 12 (Product Identification for the Receiver) = " + Global_GlobalParameters.GP12_ProductIdForReceiver + '\n';
	// Записываем в InfoFileText значение Global Parameter 13 (Model Space Scale)
	{
		char buffer[_CVTBUFSIZE];
		// Если ошибка при конвертировании double в string, то выходим из программы
		if (_gcvt_s(buffer, _CVTBUFSIZE, Global_GlobalParameters.GP13_ModelSpaceScale, Global_MaxSignifDigits)) {
			PrintErrorAndExit("  Ошибка при записи информации о Global Parameter 13 (Model Space Scale). \n\
				 Ошибка при конвертировании double в string.");
		}
		InfoFileText = InfoFileText + "Глобальный параметр 13 (Model Space Scale) = " + buffer + '\n';
	}
	// Записываем в InfoFileText значение Global Parameter 14 (Units Flag - Код единицы измерения)
	InfoFileText = InfoFileText + "Глобальный параметр 14 (Units Flag - Код единицы измерения) = " + to_string(Global_GlobalParameters.GP14_UnitsFlag) + '\n';
	// Записываем в InfoFileText значение Global Parameter 15 (Units Name)
	InfoFileText = InfoFileText + "Глобальный параметр 15 (Units Name - Имя единицы измерения) = " + Global_GlobalParameters.GP15_UnitsName + '\n';
	// Записываем в InfoFileText значение Global Parameter 16 (Maximum Number of Line Weight Gradations)
	InfoFileText = InfoFileText + "Глобальный параметр 16 (Maximum Number of Line Weight Gradations) = " + to_string(Global_GlobalParameters.GP16_MaxNumLineWeightGradations) + '\n';
	// Записываем в InfoFileText значение Global Parameter 17 (Width of Maximum Line Weight in Units)
	{
		char buffer[_CVTBUFSIZE];
		// Если ошибка при конвертировании double в string, то выходим из программы
		if (_gcvt_s(buffer, _CVTBUFSIZE, Global_GlobalParameters.GP17_MaximumLineWidth, Global_MaxSignifDigits)) {
			PrintErrorAndExit("Ошибка при записи информации о Global Parameter 17 (Maximum Line Width). \n\
				 Ошибка при конвертировании double в string.");
		}
		InfoFileText = InfoFileText + "Глобальный параметр 17 (Width of Maximum Line Weight in Units) = " + buffer + '\n';
	}
	// Записываем в InfoFileText значение Global Parameter 18 (Date and Time of Exchange File Generation)
	InfoFileText = InfoFileText + "Глобальный параметр 18 (Date and Time of Exchange File Generation (YYYYMMDD:hhmmss or YYMMDD:hhmmss)) = " + Global_GlobalParameters.GP18_DateTimeFileGeneration + '\n';
	// Записываем в InfoFileText значение Global Parameter 19 (Minimum User-Intended Resolution)
	{
		char buffer[_CVTBUFSIZE];
		// Если ошибка при конвертировании double в string, то выходим из программы
		if (_gcvt_s(buffer, _CVTBUFSIZE, Global_GlobalParameters.GP19_MinimumResolution, Global_MaxSignifDigits)) {
			PrintErrorAndExit("Ошибка при записи информации о Global Parameter 19 (Minimum Resolution). \n\
				 Ошибка при конвертировании double в string.");
		}
		InfoFileText = InfoFileText + "Глобальный параметр 19 (Minimum User-Intended Resolution) = " + buffer + '\n';
	}
	// Записываем в InfoFileText значение Global Parameter 20 (Approximate Maximum Coordinate Value)
	{
		char buffer[_CVTBUFSIZE];
		// Если ошибка при конвертировании double в string, то выходим из программы
		if (_gcvt_s(buffer, _CVTBUFSIZE, Global_GlobalParameters.GP20_MaxCoordValue, Global_MaxSignifDigits)) {
			PrintErrorAndExit("Ошибка при записи информации о Global Parameter 20 (Approximate Maximum \n\
		Coordinate Value). Ошибка при конвертировании double в string.");
		}
		InfoFileText = InfoFileText + "Глобальный параметр 20 (Approximate Maximum Coordinate Value) = " + buffer + '\n';
	}
	// Записываем в InfoFileText значение Global Parameter 21 (Name of Author)
	InfoFileText = InfoFileText + "Глобальный параметр 21 (Name of Author) = " + Global_GlobalParameters.GP21_AuthorName + '\n';
	// Записываем в InfoFileText значение Global Parameter 22 (Author’s Organization)
	InfoFileText = InfoFileText + "Глобальный параметр 22 (Author’s Organization) = " + Global_GlobalParameters.GP22_AuthorsOrganization + '\n';
	// Записываем в InfoFileText значение Global Parameter 23 (Version Flag)
	InfoFileText = InfoFileText + "Глобальный параметр 23 (Version Flag) = " + to_string(Global_GlobalParameters.GP23_VersionFlag) + '\n';
	// Записываем в InfoFileText значение Global Parameter 24 (Drafting Standard Flag)
	InfoFileText = InfoFileText + "Глобальный параметр 24 (Drafting Standard Flag) = " + to_string(Global_GlobalParameters.GP24_DraftingStandardFlag) + '\n';
	// Записываем в InfoFileText значение Global Parameter 25 (Date and Time Model was Created or Modified)
	InfoFileText = InfoFileText + "Глобальный параметр 25 (Date and Time Model was Created or Modified) = " + Global_GlobalParameters.GP25_DateTimeNativModelModified + '\n';
	// Записываем в InfoFileText значение Global Parameter 26 (Application Protocol/Subset Identifier)
	InfoFileText = InfoFileText + "Глобальный параметр 26 (Application Protocol/Subset Identifier) = " + Global_GlobalParameters.GP26_AppProtocolIdentifier + '\n';
}  // Конец функции ReadAndCheckGlobalParameters()
//
// Читает символ разделителя (Parameter Delimiter  или  Record Delimiter) начиная с символа с индексом i в разделах Global 
// Section  и  Parameter Data Section. Устанавливает значение i на символ, следующий за разделителем или за завершающими
// пробелами (если они есть). Аргумент  Section  показывает раздел ('G'- Global Section, 'P'- Parameter Data Section).
// Возвращает прочитанный символ.
char ReadDelimiters(const vector<string>& SectionVector, int& i, const char& Section)
{
	int StrLength;  // Длина строки параметров, откуда происходит чтение. Равно 72 для раздела 'G'(Global Section). Равно 64 для раздела 'P' (Parameter Data Section)
	// Проверяем аргумент Section (должен быть либо 'G', либо 'P')
	if (Section != 'G' && Section != 'P')
		PrintErrorAndExit("Ошибка в коде! Аргумент №4 в функции ReadDelimiters() отличается от \'G\' и \'P\'");
	Section == 'G' ? StrLength = Global_GSectionLineLength : StrLength = Global_PSectionLineLength;
	//  Проверяем, является ли символ разделителем 
	if (SectionVector[i / StrLength][i % StrLength] != Global_GlobalParameters.GP1_ParameterDelimiter &&
		SectionVector[i / StrLength][i % StrLength] != Global_GlobalParameters.GP2_RecordDelimiter)
	{
		SetColor(12);   cout << "\nОшибка при чтении разделителя в разделе  ";
		(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
		cout << "Символ №" << (i % StrLength) + 1 << " в строке № " << (i / StrLength) + 1 << " раздела ";
		(Section == 'G') ? cout << "Global Section  " : cout << "Parameter Data Section  ";
		cout << "не является разделителем." << endl << endl;   SetColor(7);
		cout << "   Нажмите ENTER  (выход из программы)" << endl;
		PressENTER();
		exit(0);
	}
	// Если символ является разделителем
	// Если символ является Record Delimiter, возвращаем символ  Record Delimiter (после Record Delimiter ничего не читаем)
	if (SectionVector[i / StrLength][i % StrLength] == Global_GlobalParameters.GP2_RecordDelimiter) return Global_GlobalParameters.GP2_RecordDelimiter;
	// Иначе (т.е. если символ является Parameter Delimiter)
	i++;  //  Переходим на следующий символ
	//  Пропускаем все символы 'пробел' (если они есть)
	while (SectionVector[i / StrLength][i % StrLength] == ' ') i++;
	return Global_GlobalParameters.GP1_ParameterDelimiter;  // Возвращаем символ   Parameter Delimiter
}  // конец функции ReadDelimiters()
//
// Читает и возвращает параметр типа String, начиная чтение с индекса i в разделах Global Section  и  Parameter Data
// Section. Меняет значение индекса i на символ, следующий за прочитанным текстом. Аргумент  Section  показывает раздел,
// в котором происходит чтение ('G'- Global Section, 'P'- Parameter Data Section).
string Read_String(const vector<string>& SectionVector, const char& GP1_ParameterDelimiter, const char& GP2_RecordDelimiter, int& i, const char& Section)
{
	string GP_String;  // Возвращаемое значение - Глобальный параметр типа String
	string LengthString; // Длина текста. Cюда читается число - первые цифры String-параметра перед символом 'H', т.е количество символов для чтения.
	int StrLength;  // Длина строки параметров, откуда происходит чтение. Равно 72 для раздела 'G'(Global Section). Равно 64 для раздела 'P' (Parameter Data Section)
	// Проверяем аргумент Section
	if (Section != 'G' && Section != 'P')
		PrintErrorAndExit("Ошибка в коде! Аргумент №4 в функции Read_String_From_GSection() не \'G\' и не \'P\'");
	Section == 'G' ? StrLength = Global_GSectionLineLength : StrLength = Global_PSectionLineLength;
	// Читаем в строку LengthString число в начале String-параметра перед символом 'H' (т.е. читаем длину строки)
	while (true) {
		if (SectionVector[i / StrLength][i % StrLength] >= '0' && SectionVector[i / StrLength][i % StrLength] <= '9' &&    // Если текущий и следующий символ - цифры, то ... 
			SectionVector[i / StrLength][i % StrLength + 1] >= '0' && SectionVector[i / StrLength][i % StrLength + 1] <= '9')
		{
			LengthString += SectionVector[i / StrLength][i % StrLength];   //  ... то записываем текущий символ (цифру) в строку LengthString, и ...
			i++;  // ... и переходим на следующий символ
		}
		else if (SectionVector[i / StrLength][i % StrLength] >= '0' && SectionVector[i / StrLength][i % StrLength] <= '9' &&    // Если текущий символ - цифра, и ...
			SectionVector[i / StrLength][i % StrLength + 1] == 'H')    // ... и следующий символ - 'H',  то ...
		{
			LengthString += SectionVector[i / StrLength][i % StrLength];   //  ... то записываем текущий символ (цифру), и ...
			i += 2;  // ... переходим на символ, следующий за символом 'H',  и ...
			break;  // ... прекращаем читать число в строку LengthString.
		}
		else {  //  иначе - сообщение об ошибке и выход из программы
			SetColor(12);   cout << "\nОшибка при чтении числа в начале String-параметра." << endl;
			cout << "Символ №" << (i % StrLength) + 1 << " в строке № " << (i / StrLength) + 1 << " раздела ";
			(Section == 'G') ? cout << "Global Section  " : cout << "Parameter Data Section  ";
			cout << "не является цифрой." << endl << endl;   SetColor(7);
			cout << "   Нажмите ENTER  (выход из программы)" << endl;
			PressENTER();
			exit(0);
		}
	}
	// Проверяем записанное число (длину строки) - не должно быть равно 0. 
	if (stoi(LengthString) == 0) {  // Если указана длина строки = 0
		SetColor(12);   cout << "\nОшибка! Число в начале String-параметра (перед символом \'H'\) равно = 0." << endl;
		cout << "Неверный символ №" << ((i - 2) % StrLength) + 1 << " в строке № " << ((i - 2) / StrLength) + 1 << endl << endl;   SetColor(7);
		cout << "   Нажмите ENTER  (выход из программы)" << endl;
		PressENTER();
		exit(0);
	}
	// Читаем текст параметра (т.е. текст после символа 'H')
	for (int j = 0; j < stoi(LengthString); j++) {
		// Если символ из допустимого диапазона, то читаем его.
		if (!(SectionVector[i / StrLength][i % StrLength] >= 0 && SectionVector[i / StrLength][i % StrLength] < 32) &&
			SectionVector[i / StrLength][i % StrLength] != 127)
		{
			GP_String += SectionVector[i / StrLength][i % StrLength];
			i++;
		}
		else {  // иначе (т.е. если символ не из допустимого диапазона) - сообщение об ошибке и выход из программы
			SetColor(12);   cout << "\nОшибка! Неверный символ String-параметра в разделе  ";
			(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
			cout << "Неверный символ №" << (i % StrLength) + 1 << " в строке № " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
			cout << "   Нажмите ENTER  (выход из программы)" << endl;
			PressENTER();
			exit(0);
		}
	}
	return GP_String;
}   // Read_String()
//
// Читает и возвращает целое число начиная чтение с индекса i в разделах Global Section  и  Parameter Data Section.
//  Меняет значение индекса i на символ, следующий за прочитанным текстом. Аргумент  Section  показывает раздел,
//  в котором происходит чтение ('G'- Global Section, 'P'- Parameter Data Section).
long long  ReadIntegerDataType(const vector<string>& SectionVector, int& i, const char& Section)
{
	string Txt;  // Текст, куда сначала читается число перед конвертацией в long long
	int StrLength;  // Длина строки параметров, откуда происходит чтение. Равно 72 для раздела 'G'(Global Section). Равно 64 для раздела 'P' (Parameter Data Section)
	// Проверяем аргумент Section
	if (Section != 'G' && Section != 'P')
		PrintErrorAndExit("Ошибка в коде! Аргумент №3 в функции ReadInteger() не \'G\' и не \'P\'");
	Section == 'G' ? StrLength = Global_GSectionLineLength : StrLength = Global_PSectionLineLength;
	// Читаем первый символ - знак числа (если он есть)
	if (SectionVector[i / StrLength][i % StrLength] == '+' || SectionVector[i / StrLength][i % StrLength] == '-') {
		Txt = SectionVector[i / StrLength][i % StrLength];
		i++;
	}
	// Проверяем, что первый символ - цифра
	if (SectionVector[i / StrLength][i % StrLength] < '0' || SectionVector[i / StrLength][i % StrLength] > '9') {
		SetColor(12);   cout << "\nОшибка! Неверный символ при чтении числа в разделе  ";
		(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
		cout << "Неверный символ №" << (i % StrLength) + 1 << " в строке № " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
		cout << "   Нажмите ENTER  (выход из программы)" << endl;
		PressENTER();
		exit(0);
	}
	// Читаем цифры
	while (SectionVector[i / StrLength][i % StrLength] >= '0' && SectionVector[i / StrLength][i % StrLength] <= '9') {
		Txt = Txt + SectionVector[i / StrLength][i % StrLength];
		i++;
	}
	return stoll(Txt);
}  // конец функции  ReadIntegerDataType()
//
// Читает и возвращает число с плавающей точкой начиная чтение с индекса i в разделах Global Section  и  Parameter Data 
// Section. Меняет значение индекса i на символ, следующий за прочитанным текстом. Аргумент  Section  показывает раздел, 
// в котором происходит чтение ('G'- Global Section, 'P'- Parameter Data Section).
double ReadRealDataType(const vector<string>& SectionVector, int& i, const char& Section)
{
	double Result;  // Возвращаемое значение
	int SignificantDigitsNumber{ 0 };  // Количество цифр в мантиссе без учёта стоящих в начале нулей. Отсчёт начинается с первой значащей цифры 
	bool DecimalPointExists{ false };  // Индикатор существования десятичной точки. false - точка не существует, true - существует.  
	string RealNumber = "";
	unsigned int StrLength{ 0 };  // Длина строки параметров, откуда происходит чтение. Равно 72 для раздела 'G'(Global Section). Равно 64 для раздела 'P' (Parameter Data Section)
	// Проверяем аргумент Section
	if (Section != 'G' && Section != 'P')
		PrintErrorAndExit("Ошибка в коде! Аргумент №3 в функции ReadRealDataType не \'G\' и не \'P\'");
	else Section == 'G' ? StrLength = Global_GSectionLineLength : StrLength = Global_PSectionLineLength;
	// Читаем первый символ мантиссы - знак числа (если он есть)
	if (SectionVector[i / StrLength][i % StrLength] == '+') i++;  // Если первый символ '+', то его пропускаем
	else if (SectionVector[i / StrLength][i % StrLength] == '-') { // Если первый символ '-', то записываем его
		RealNumber = '-';
		i++;
	}
	// Прочитали знак числа '+'или'-' (если он был). Далее - целая часть мантиссы. Это может быть цифра от 0 до 9 или '.'. Проверяем, иначе ошибка и выход из программы
	else if ((SectionVector[i / StrLength][i % StrLength] < '0' || SectionVector[i / StrLength][i % StrLength] > '9') &&
		SectionVector[i / StrLength][i % StrLength] != '.') {
		SetColor(12);   cout << "\nОшибка! Неверный первый символ числа типа Real  \nв разделе  ";
		(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
		cout << "Неверный символ №" << (i % StrLength) + 1 << " в строке № " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
		cout << "   Нажмите ENTER  (выход из программы)" << endl;
		PressENTER();
		exit(0);
	}
	// Читаем мантиссу числа - это может быть цифра от 0 до 9 или десятичная точка '.' (десятичная точка может быть не более 1 раза)
	while (true) {
		if (SignificantDigitsNumber == 0 && SectionVector[i / StrLength][i % StrLength] == '0' &&     // Пропускаем символы '0' в начале числа до десятичной точки
			DecimalPointExists == false) {
			i++;
			continue;
		}
		else if (SectionVector[i / StrLength][i % StrLength] >= '0' && SectionVector[i / StrLength][i % StrLength] <= '9') {  // Читаем цифры - от 0 до 9 включительно
			RealNumber = RealNumber + SectionVector[i / StrLength][i % StrLength];
			if (SectionVector[i / StrLength][i % StrLength] == '0' && SignificantDigitsNumber == 0) {}
			else SignificantDigitsNumber++;
			i++;
			continue;
		}
		else if (SectionVector[i / StrLength][i % StrLength] == '.') {  // Читаем десятичную точку '.'
			if (DecimalPointExists == true) {    // Если встретилась вторая десятичная точка - то ошибка и выход
				SetColor(12);   cout << "\nОшибка при чтении числа типа Real  в разделе  ";
				(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
				cout << "В мантиссе числа есть более одной десятичной точки." << endl;
				cout << "Неверный символ №" << (i % StrLength) + 1 << " в строке № " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
				cout << "   Нажмите ENTER  (выход из программы)" << endl;
				PressENTER();
				exit(0);
			}
			else if (SignificantDigitsNumber == 0) RealNumber = RealNumber + '0';   // Если целая часть мантиссы отсутствует или равна 0, то добавляем '0' перед десятичной точкой 
			DecimalPointExists = true;
			RealNumber = RealNumber + ',';
			i++;
			continue;
		}
		// Проверяем что наступил конец мантиссы: если символ не точка '.' и не цифра, то должно быть 'E', 'e', 'D', 'd' или  разделитель
		if (SectionVector[i / StrLength][i % StrLength] == 'E' || SectionVector[i / StrLength][i % StrLength] == 'e' ||
			SectionVector[i / StrLength][i % StrLength] == 'D' || SectionVector[i / StrLength][i % StrLength] == 'd' ||
			SectionVector[i / StrLength][i % StrLength] == Global_GlobalParameters.GP1_ParameterDelimiter ||
			SectionVector[i / StrLength][i % StrLength] == Global_GlobalParameters.GP2_RecordDelimiter) {
		}
		else {
			SetColor(12);   cout << "\nОшибка при чтении числа типа Real в разделе  ";
			(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
			cout << "Ошибка при чтении мантиссы числа. Неверный символ №" << (i % StrLength) + 1 << " в строке № " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
			cout << "   Нажмите ENTER  (выход из программы)" << endl;
			PressENTER();
			exit(0);
		}
		break;  // Выход из цикла чтения мантиссы и продолжение чтения
	}   //  Закончили читать мантиссу числа
	// Для числа с десятичной точкой удаляем нули в конце мантиссы после десятичной точки   
	if (DecimalPointExists == true) {
		while (RealNumber[RealNumber.size() - 1] == '0') {
			RealNumber.resize(RealNumber.size() - 1);
			SignificantDigitsNumber--;
		}
	}
	// Читаем экспоненту - правую часть числа после символа 'E', 'e', 'D' или 'd', если она есть
	if (SectionVector[i / StrLength][i % StrLength] == 'E' || SectionVector[i / StrLength][i % StrLength] == 'e' ||
		SectionVector[i / StrLength][i % StrLength] == 'D' || SectionVector[i / StrLength][i % StrLength] == 'd') {
		RealNumber = RealNumber + 'E';
		i++;
		// Читаем знак (если он есть) перед экспонентой 
		if (SectionVector[i / StrLength][i % StrLength] == '+' || SectionVector[i / StrLength][i % StrLength] == '-') {
			RealNumber = RealNumber + SectionVector[i / StrLength][i % StrLength];
			i++;
		}
		// Проверяем что первый символ экспоненты - цифра
		if (SectionVector[i / StrLength][i % StrLength] < '0' || SectionVector[i / StrLength][i % StrLength] > '9') {
			SetColor(12);   cout << "\nОшибка! Неверный символ при чтении цифр экспоненты числа \nв разделе  ";
			(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
			cout << "Неверный символ №" << (i % StrLength) + 1 << " в строке № " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
			cout << "   Нажмите ENTER  (выход из программы)" << endl;
			PressENTER();
			exit(0);
		}
		// Читаем цифры экспоненты
		while (SectionVector[i / StrLength][i % StrLength] >= '0' && SectionVector[i / StrLength][i % StrLength] <= '9') {
			RealNumber = RealNumber + SectionVector[i / StrLength][i % StrLength];
			i++;
		}
		// Проверяем что наступил конец экспоненты: если символ не разделитель - то ошибка и выход     
		if (SectionVector[i / StrLength][i % StrLength] == Global_GlobalParameters.GP1_ParameterDelimiter ||
			SectionVector[i / StrLength][i % StrLength] == Global_GlobalParameters.GP2_RecordDelimiter) {
		}
		else {
			SetColor(12);   cout << "\nОшибка при чтении числа типа Real в разделе ";
			(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
			cout << "Ошибка при чтении экспоненты числа. Неверный символ №" << (i % StrLength) + 1 << " в строке № " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
			cout << "   Нажмите ENTER  (выход из программы)" << endl;
			PressENTER();
			exit(0);
		}

	}
	// Формируем результат
	Result = atof(RealNumber.c_str());
	// Если происходит потеря точности - печатаем сообщение об этом
	if (SignificantDigitsNumber > Global_MaxSignifDigits) {
		char buffer[_CVTBUFSIZE];
		if (_gcvt_s(buffer, _CVTBUFSIZE, Result, Global_MaxSignifDigits)) {  // Если ошибка при конвертировании double в string, то выходим из программы
			PrintErrorAndExit("Ошибка в ReadRealDataType() при вызове _gcvt_s() при потере точности.");
		}
		SetColor(12);   cout << "\nВнимание!";  SetColor(14);   cout << "  Потеря точности при чтении числа с плавающей запятой." << endl;
		cout << "Положение последнего символа числа:  раздел  ";  (Section == 'G') ? cout << "Global Section,\n" : cout << "Parameter Data Section,\n";
		cout << "строка № " << (i / StrLength) + 1 << ", символ №" << (i % StrLength) << endl;
		cout << "В дальнейших расчётах вместо числа  " << RealNumber << endl;
		cout << "будет использоваться число  " << buffer << "." << endl << endl;  SetColor(7);
	}
	return Result;
}   //  конец функции  ReadRealDataType()
//
// 
// Читает и возвращает целое число в разделе Directory Entry для объекта на строке D_Sect_Str_Number в поле FieldNumber.
// Если поле заполнено пробелами - возвращает 0. Если в поле находится текст вместо целого числа - ошибка и выход из программы
int ReadIntegerIn_D_Section(const vector<string>& DSectionVector, const int D_Sect_Str_Number, const int FieldNumber)
{
	// static int N{ 0 };    // Удалить
	// N++;   // Удалить
	// cout << "ReadIntegerIn_D_Section().  N= " << N << ". D_Sect_Str_Number= " << D_Sect_Str_Number << ". FieldNumber= " << FieldNumber;   // Удалить
	if (D_Sect_Str_Number % 2 == 0) {    // Если указанный номер строки D_Sect_Str_Number - чётное число, то ошибка и выход из программы
		string txt = "\nОшибка при вызове функции ReadIntegerIn_D_Section().\nНеверный второй аргумент D_Sect_Str_Number. Получено значение " + to_string(D_Sect_Str_Number) +
			" \n(значение должно быть нечётным числом).";
		// cout << "\nFieldNumber = " << FieldNumber << endl;   //  Удалить
		PrintErrorAndExit(txt);
	}
	int Result;  // Возвращаемое значение
	int DSectionStringIndex = D_Sect_Str_Number - 1 + static_cast<int>(FieldNumber / 10); // Индекс строки в векторе DSectionVector, где нужно прочитать число
	// cout << "Функция ReadIntegerIn_D_Section().  DSectionStringIndex = " << DSectionStringIndex << endl;   // Удалить
	int StartIndex = (Global_D_Section_Field_Size * (FieldNumber - 1)) % Global_IGES_File_StringLength;  // Индекс символа с начала строки, с которого нужно начать чтение числа
	// cout << "Функция ReadIntegerIn_D_Section().  StartIndex = " << StartIndex << endl;   // Удалить
	string TxtInField = DSectionVector[DSectionStringIndex].substr(StartIndex, Global_D_Section_Field_Size);  // Читаем в строку TxtInField текст из указанного поля
	// cout << "Функция ReadIntegerIn_D_Section().  TxtInField = " << TxtInField << endl;   // Удалить   
	if (TxtInField == "        ") {
		// cout << " Result= 8_пробелов. " << endl;   // Удалить
		return 0;   // Если поле состоит из 8-ми пробелов - возвращаем 0 
	}
	// Проверяем, что все 8 символов в TxtInField - это целое число (со знаком или без)
	{
		int i = 0;
		for (; i < 8; i++) {   // пропускаем пробелы в начале
			if (TxtInField[i] != ' ') break;
		}
		if ((i < 7 && TxtInField[i] == '+') || (i < 7 && TxtInField[i] == '-')) i++;  // Пропускаем знак + или - (если он есть)
		for (; i < 8; i++) {   // проверяем, что символы - это цифры от 0 до 9
			if (TxtInField[i] < '0' || TxtInField[i] > '9') break;
		}
		if (i < 8) {  // Если встретили посторонний символ
			string txt = "\nОшибка в исходном файле: \nв Directory Entry Section неверное значение у элемента на строке " +
				to_string(D_Sect_Str_Number) + ", в поле " + to_string(FieldNumber) + ".\nПоле содержит текст \"" + TxtInField + "\" (должно быть целое число у правого края поля).";
			PrintErrorAndExit(txt);
		}
	}
	try {
		Result = stoi(DSectionVector[DSectionStringIndex].substr(StartIndex, Global_D_Section_Field_Size));
	}
	catch (invalid_argument const& ex) {
		string txt = "\nОшибка в исходном файле: \nв Directory Entry Section неверное значение у элемента на строке " +
			to_string(D_Sect_Str_Number) + ", в поле " + to_string(FieldNumber) + ".\nПоле содержит текст \"" + TxtInField + "\" (должно быть целое число).";
		PrintErrorAndExit(txt);
	}
	// cout << " Result= " << Result << endl;   // Удалить
	return Result;
}
//
// 
// Читает цвет (т.е. расшифровывает поле 13 - Color Number) для любого Entity на строке Entity_D_Sect_Str_Number. 
// Возвращает: "стандарный" цвет (0-8), или нестандартный цвет - 10-значное число формата 1RGB (например для  
// RGB:254,001,220 это 1254001220). "Стандарные": 0-no color(default), 1-Black(RGB:0,0,0), 2-Red (RGB:255,0,0), 3-Green  
// (RGB:0,255,0), 4-Blue(RGB:0,0,255), 5-Yellow(RGB:255,255,0), 6-Magenta (RGB:255,0,255), 7-Cyan (RGB:0,255,255), 8-White (RGB:255,255,255).
int ReadEntityColor(const vector<string>& D_SectionVector, const vector<string>& P_SectionVector, const int D_Sect_Str_Number)
{
	int EntityField13_ColorNumber = ReadIntegerIn_D_Section(D_SectionVector, D_Sect_Str_Number, 13);  // Число в поле 13 (Color Number).
		// stoi(D_SectionVector[D_Sect_Str_Number].substr(16, 8)); // Число в поле 13 (Color Number).    Удалить
	int ColorField2_ParameterDataLine;  // Число в поле 2 для объекта Type 314 "Color Entity" - указатель на строку в Parameter Data для Color
	int ColorTypeNumber_D_Section{ 0 };  // Номер объекта по ссылке (должен быть 314 "Color Definition Entity") в разделе D_Section
	int ColorTypeNumber_P_Section{ 0 };  // Номер объекта по ссылке (должен быть 314 "Color Definition Entity") в разделе P_Section
	double ColorCoordinate1_RED{ 0 }, ColorCoordinate2_GREEN{ 0 }, ColorCoordinate3_BLUE{ 0 };  // Прочитанные из P_Section для Color значения RGB
	int Color_R{ 0 }, Color_G{ 0 }, Color_B{ 0 };  // Значения RGB для цвета (от 0 до 255)
	// Если в поле 13 число больше 8 или отрицательное с модулем больше размера Directory Entry, то сообщение об ошибке и выход из программы
	if (EntityField13_ColorNumber > 8 || (EntityField13_ColorNumber < (-1) * static_cast<int>(D_SectionVector.size()))) {
		string txt = "\nОшибка в исходном файле:\n  Directory Entry Section, объект на строке " + to_string(D_Sect_Str_Number) +
			", значение в поле 13 \"Color Number\" = " + to_string(EntityField13_ColorNumber) +
			".\nДолжно быть меньше/равно 8 или отрицательным по модулю меньше размера Directory Entry.";
		PrintErrorAndExit(txt);
	}
	else if (EntityField13_ColorNumber >= 0 && EntityField13_ColorNumber <= 8) return EntityField13_ColorNumber;  // Возвращаем номер цвета если у Entity в поле 13 указан "стандартный" цвет
	// Иначе (т.е. если в поле 13 отрицательное число) - читаем цвет в объекте Type 314 по ссылке на строке "-Field13"
	ColorTypeNumber_D_Section = ReadIntegerIn_D_Section(D_SectionVector, -EntityField13_ColorNumber, 1);
	// stoi(D_SectionVector[-EntityField13_ColorNumber - 1].substr(0, 8));    // Удалить
// Если поле 1 по указанной ссылке НЕ объект 314 - то Сообщение об ошибке и выход из программы
	if (ColorTypeNumber_D_Section != Global_Entity314ColorNumber) {
		string txt = "\nОшибка в исходном файле:\n  Directory Entry Section, объект на строке " + to_string(-EntityField13_ColorNumber) +
			" имеет номер " + to_string(ColorTypeNumber_D_Section) + " (должен быть 314).";
		PrintErrorAndExit(txt);
	}
	// Иначе (если ссылка указывает на объект Entity 314) - читаем RGB для цвета в Parameter Data Section
	// В поле 2 объекта 314 "Color" читаем Pointer, указывающий на строку в Parameter Data Section
	ColorField2_ParameterDataLine = ReadIntegerIn_D_Section(D_SectionVector, -EntityField13_ColorNumber, 2);
	// stoi(D_SectionVector[-EntityField13_ColorNumber - 1].substr(8, 8));     // Удалить
// Проверка значения в поле 2 для объекта Entity 314.  Значение должно быть больше 0 и меньше/равно размера Parameter Data Section
	if (ColorField2_ParameterDataLine < 1 || ColorField2_ParameterDataLine > P_SectionVector.size()) {
		string txt = "\nОшибка в исходном файле:\nраздел Directory Entry Section, ошибка в поле 2 на строке " + to_string(-EntityField13_ColorNumber) +
			".\nУказано значение " + to_string(ColorField2_ParameterDataLine) + " (должно быть более 0 и менее/равно размера Parameter Data Section).";
		PrintErrorAndExit(txt);
	}
	//  --- Читаем RGB для цвета в Parameter Data Section --- 
	int i = (ColorField2_ParameterDataLine - 1) * Global_PSectionLineLength;  // Положение "курсора" в Parameter Data Section
	// Читаем первый аргумент в Parameter Data Section - Entity Type Number. Должно быть 314.
	ColorTypeNumber_P_Section = ReadIntegerDataType(P_SectionVector, i, 'P');  // Читаем Entity Type Number
	// Проверяем первый аргумент - Entity Type Number (должно быть 314)
	if (ColorTypeNumber_P_Section != 314) {
		string txt = "\nОшибка в исходном файле:\nв разделе Parameter Data Section неверный Entity Type Number. В строке " + to_string(ColorField2_ParameterDataLine) +
			" указано \nзначение " + to_string(ColorTypeNumber_P_Section) + " (должно быть 314).";
		PrintErrorAndExit(txt);
	}
	// Читаем разделитель после первого аргумента в Parameter Data Section (т.е. после Entity Type Number)
	if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
		string txt = "Ошибка в исходном файле! В разделе Parameter Data Section в строке " + to_string(ColorField2_ParameterDataLine) + ".\n";
		txt = txt + "Неверный разделитель после первого аргумента (должен быть символ  " + Global_GlobalParameters.GP1_ParameterDelimiter + " ).";
		PrintErrorAndExit(txt);
	}
	// Читаем второй аргумент - First color coordinate (red)
	ColorCoordinate1_RED = ReadRealDataType(P_SectionVector, i, 'P');
	// Проверяем второй аргумент - First color coordinate (red) (должно быть от 0 до 100 включительно)
	if (ColorCoordinate1_RED < 0. || ColorCoordinate1_RED > 100.) {
		string txt = "\nОшибка в исходном файле:\nВ разделе Parameter Data Section, в строке " + to_string(ColorField2_ParameterDataLine) +
			" второй аргумент = " + to_string(ColorCoordinate1_RED) + "\n(должно быть от 0 до 100 включительно).";
		PrintErrorAndExit(txt);
	}
	// Читаем разделитель после второго аргумента в Parameter Data Section, т.е. после First color coordinate (red)
	if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
		string txt = "Ошибка! В разделе Parameter Data Section в строке " + to_string(ColorField2_ParameterDataLine) + ".\n";
		txt = txt + "Неверный разделитель после второго аргумента (должен быть символ  " + Global_GlobalParameters.GP1_ParameterDelimiter + " ).";
		PrintErrorAndExit(txt);
	}
	// Читаем третий аргумент - Second color coordinate (green)
	ColorCoordinate2_GREEN = ReadRealDataType(P_SectionVector, i, 'P');
	// Проверяем третий аргумент - Second color coordinate (green) (должно быть от 0 до 100 включительно)
	if (ColorCoordinate2_GREEN < 0. || ColorCoordinate2_GREEN > 100.) {
		string txt = "\nОшибка в исходном файле:\nВ разделе Parameter Data Section, в строке " + to_string(ColorField2_ParameterDataLine) +
			" третий аргумент = " + to_string(ColorCoordinate2_GREEN) + "\n(должно быть от 0 до 100 включительно).";
		PrintErrorAndExit(txt);
	}
	// Читаем разделитель после третьего аргумента в Parameter Data Section, т.е. после Second color coordinate (green)
	if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
		string txt = "Ошибка в исходном файле! В разделе Parameter Data Section в строке " + to_string(ColorField2_ParameterDataLine) + ".\n";
		txt = txt + "Неверный разделитель после третьего аргумента (должен быть символ  " + Global_GlobalParameters.GP1_ParameterDelimiter + " ).";
		PrintErrorAndExit(txt);
	}
	// Читаем четвёртый аргумент - Third color coordinate (blue)
	ColorCoordinate3_BLUE = ReadRealDataType(P_SectionVector, i, 'P');
	// Проверяем четвёртый аргумент - Third color coordinate (blue) (должно быть от 0 до 100 включительно)
	if (ColorCoordinate3_BLUE < 0. || ColorCoordinate3_BLUE > 100.) {
		string txt = "\nОшибка в исходном файле:\nВ разделе Parameter Data Section, в строке " + to_string(ColorField2_ParameterDataLine) +
			" четвёртый аргумент = " + to_string(ColorCoordinate3_BLUE) + "\n(должно быть от 0 до 100 включительно).";
		PrintErrorAndExit(txt);
	}
	// Читаем разделитель после четвёртого аргумента в Parameter Data Section, т.е. после Third color coordinate (blue)
	if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
		string txt = "Ошибка! В разделе Parameter Data Section в строке " + to_string(ColorField2_ParameterDataLine) + ".\n";
		txt = txt + "Неверный разделитель после четвёртого аргумента (должен быть символ  " + Global_GlobalParameters.GP1_ParameterDelimiter + " ).";
		PrintErrorAndExit(txt);
	}
	// -- Расчитываем значения RGB --
	// Расчитываем Color_R
	static_cast<int>(255 * ColorCoordinate1_RED / 100) == static_cast<int>(255 * ColorCoordinate1_RED / 100 + 0.5) ?
		Color_R = static_cast<int>(255 * ColorCoordinate1_RED / 100) : Color_R = static_cast<int>(255 * ColorCoordinate1_RED / 100 + 0.5);
	// Расчитываем Color_G
	static_cast<int>(255 * ColorCoordinate2_GREEN / 100) == static_cast<int>(255 * ColorCoordinate2_GREEN / 100 + 0.5) ?
		Color_G = static_cast<int>(255 * ColorCoordinate2_GREEN / 100) : Color_G = static_cast<int>(255 * ColorCoordinate2_GREEN / 100 + 0.5);
	// Расчитываем Color_B
	static_cast<int>(255 * ColorCoordinate3_BLUE / 100) == static_cast<int>(255 * ColorCoordinate3_BLUE / 100 + 0.5) ?
		Color_B = static_cast<int>(255 * ColorCoordinate3_BLUE / 100) : Color_B = static_cast<int>(255 * ColorCoordinate3_BLUE / 100 + 0.5);
	// --- Возвращаем значение цвета  ---
	if (Color_R == 0 && Color_G == 0 && Color_B == 0) return 1;  // RGB 0,0,0 - цвет 1 - Чёрный
	if (Color_R == 255 && Color_G == 0 && Color_B == 0) return 2;  // RGB 255,0,0 - цвет 2 - Красный
	if (Color_R == 0 && Color_G == 255 && Color_B == 0) return 3;  // RGB 0,255,0 - цвет 3 - Зелёный
	if (Color_R == 0 && Color_G == 0 && Color_B == 255) return 4;  // RGB 0,0,255 - цвет 4 - Синий
	if (Color_R == 255 && Color_G == 255 && Color_B == 0) return 5;  // RGB 255,255,0 - цвет 5 - Жёлтый
	if (Color_R == 255 && Color_G == 0 && Color_B == 255) return 6;  // RGB 255,0,255 - цвет 6 - Magenta (Пурпурный)
	if (Color_R == 0 && Color_G == 255 && Color_B == 255) return 7;  // RGB 0,255,255 - цвет 7 - Cyan (Бирюзовый)
	if (Color_R == 255 && Color_G == 255 && Color_B == 255) return 8;  // RGB 255,255,255 - цвет 8 - Белый
	else return 1000000000 + Color_R * 1000000 + Color_G * 1000 + Color_B;
}   // конец функции ReadEntityColor()
//
//
// Читает "Отрезок" (Line Entity - Type 110) в Directory Entry Section на строке D_Sect_Str_Number.
// Обрабатывает и записывает информацию в вектор Global_Entity110_Line_Vector. 
void ReadEntity110_Line(const vector<string>& D_SectionVector, const vector<string>& P_SectionVector, const int D_Sect_Str_Number)
{
	Entity110_Line TMPEntity110_Line;  // Временная структура для записи данных об Отрезке. 
	int Field2_ParameterDataLine = ReadIntegerIn_D_Section(D_SectionVector, D_Sect_Str_Number, 2);  // Число в поле 2 (Parameter Data Pointer)
		// stoi(D_SectionVector[D_Sect_Str_Number-1].substr(8, 8)); // Число в поле 2 (Parameter Data Pointer)  // Удалить
	int Field7_PointerTransformMatrix = ReadIntegerIn_D_Section(D_SectionVector, D_Sect_Str_Number, 7);  // Число в поле 7 (Pointer to the Directory Entry of a Transformation Matrix (Type 124))
		// stoi(D_SectionVector[D_Sect_Str_Number-1].substr(48, 8));  // Число в поле 7 (Pointer to the Directory Entry of a Transformation Matrix (Type 124))    Удалить
	int Field15_FormNumber;  //  Поле 15 в Directory Entry. 0 - отрезок. 1 - луч с началом в (X1,Y1,Z1). 2 - бесконечная прямая
	int TypeNumber{ 0 };  // Номер объекта (должен быть 110 "Line") - для проверки параметра 1 в Parameter Data Section
	TMPEntity110_Line.D_Section_String_Number = D_Sect_Str_Number;
	TMPEntity110_Line.P_Section_String_Number = Field2_ParameterDataLine;
	// --- Читаем координаты отрезка в Parameter Data Section ---
	{
		int i = (Field2_ParameterDataLine - 1) * Global_PSectionLineLength;   // Индекс читаемого символа в Parameter Data Section
		// Читаем первый аргумент в Parameter Data Section - Entity Type Number. Должно быть 110.
		TypeNumber = ReadIntegerDataType(P_SectionVector, i, 'P');
		// Проверяем Entity Type Number (должно быть 110)
		if (TypeNumber != Global_Entity110LineNumber) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section неверный Entity Type Number.\nВ строке " + to_string(Field2_ParameterDataLine) +
				" указано значение " + to_string(TypeNumber) + " (должно быть 110).";
			PrintErrorAndExit(txt);
		}
		// Читаем разделитель после первого аргумента в Parameter Data Section (т.е. после Entity Type Number)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после первого аргумента\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем второй аргумент в Parameter Data Section - X1.
		TMPEntity110_Line.X1origin = ReadRealDataType(P_SectionVector, i, 'P');
		// Читаем разделитель после второго аргумента в Parameter Data Section (т.е. после X1)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после второго аргумента\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем третий аргумент в Parameter Data Section - Y1.
		TMPEntity110_Line.Y1origin = ReadRealDataType(P_SectionVector, i, 'P');
		// Читаем разделитель после третьего аргумента в Parameter Data Section (т.е. после Y1)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после третьего аргумента\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем четвёртый аргумент в Parameter Data Section - Z1.
		TMPEntity110_Line.Z1origin = ReadRealDataType(P_SectionVector, i, 'P');
		// Читаем разделитель после четвёртого аргумента в Parameter Data Section (т.е. после Z1)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после четвёртого аргумента\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем пятый аргумент в Parameter Data Section - X2.
		TMPEntity110_Line.X2origin = ReadRealDataType(P_SectionVector, i, 'P');
		// Читаем разделитель после пятого аргумента в Parameter Data Section (т.е. после X2)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после пятого аргумента\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем шестой аргумент в Parameter Data Section - Y2.
		TMPEntity110_Line.Y2origin = ReadRealDataType(P_SectionVector, i, 'P');
		// Читаем разделитель после шестого аргумента в Parameter Data Section (т.е. после Y2)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после шестого аргумента\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем седьмой аргумент в Parameter Data Section - Z2.
		TMPEntity110_Line.Z2origin = ReadRealDataType(P_SectionVector, i, 'P');
	}
	// Рассчитываем длину отрезка
	TMPEntity110_Line.Length = sqrt((TMPEntity110_Line.X2origin - TMPEntity110_Line.X1origin) * (TMPEntity110_Line.X2origin - TMPEntity110_Line.X1origin) +
		(TMPEntity110_Line.Y2origin - TMPEntity110_Line.Y1origin) * (TMPEntity110_Line.Y2origin - TMPEntity110_Line.Y1origin) +
		(TMPEntity110_Line.Z2origin - TMPEntity110_Line.Z1origin) * (TMPEntity110_Line.Z2origin - TMPEntity110_Line.Z1origin));
	// Читаем поле 15 - Form Number. Проверяем корректность значения - должно быть 0, 1 или 2.
	Field15_FormNumber = ReadIntegerIn_D_Section(D_SectionVector, D_Sect_Str_Number, 15);
	//  stoi(D_SectionVector[D_Sect_Str_Number].substr(32, 8));        Удалить
	if (Field15_FormNumber < 0 || Field15_FormNumber > 2) {
		string txt = "Ошибка в исходном файле!\nВ разделе Directory Entry неверное значение в поле 15 элемента в строке " + to_string(D_Sect_Str_Number) +
			".\nУказано значение " + to_string(Field15_FormNumber) + " (значение должно быть 0, 1 или 2).";
		PrintErrorAndExit(txt);
	}
	TMPEntity110_Line.FormNumber = Field15_FormNumber;
	// --- Читаем цвет отрезка ---
	TMPEntity110_Line.Color = ReadEntityColor(D_SectionVector, P_SectionVector, D_Sect_Str_Number);
	// --- Читаем матрицы трансформации (указанную матрицу и все входящие в неё матрицы (если такие есть))---
	ReadEntity124_TrMatrix(TMPEntity110_Line.TransformationMatrixVector, D_SectionVector, P_SectionVector, Field7_PointerTransformMatrix);
	// Преобразуем координаты точек через матрицы трансформации
	TransformPointByTransfMatrix(TMPEntity110_Line.X1origin, TMPEntity110_Line.Y1origin, TMPEntity110_Line.Z1origin,
		TMPEntity110_Line.X1, TMPEntity110_Line.Y1, TMPEntity110_Line.Z1, TMPEntity110_Line.TransformationMatrixVector);
	TransformPointByTransfMatrix(TMPEntity110_Line.X2origin, TMPEntity110_Line.Y2origin, TMPEntity110_Line.Z2origin,
		TMPEntity110_Line.X2, TMPEntity110_Line.Y2, TMPEntity110_Line.Z2, TMPEntity110_Line.TransformationMatrixVector);
	//  Добавляем прочитанный отрезок в вектор отрезков
	Global_Entity110_Line_Vector.push_back(TMPEntity110_Line);
}  // конец функции ReadEntity110_Line()
//
//
// Читает матрицу трансформации (Transformation Matrix Entity - Type 124) в Directory Entry Section, которая
// находится на строке Entity124_D_Sect_Str_Number, а также читает все входящие в неё матрицы (если такие есть). Записывает результат в TransformationMatrixVector.
void ReadEntity124_TrMatrix(vector<Entity124_TransformationMatrix>& TransformationMatrixVector, const vector<string>& D_SectionVector,
	const vector<string>& P_SectionVector, int Entity124_D_Sect_Str_Number)
{
	if (Entity124_D_Sect_Str_Number == 0) return;
	int EntityNumber;  // Поле 1 в Directory Entry, или первый аргумент в Parameter Data - для сравнения с правильным значением (Global_Entity124TransMatrixNumber)
	int Field2_ParameterDataLine;  // Число в поле 2 - указатель (Pointer) на строку в Parameter Data
	int Field7_TrMatrixPointer;  // Число в поле 7 (Указатель на вложенную Transformation Matrix). 
	int Field15_FormNumber;  // Должно быть 0 или 1
	Entity124_TransformationMatrix TmpEntity124_TrMatrix;  // Временная структура для чтения мартицы трансформации. После заполнения добавляется к TransformationMatrixVector
	// Читаем и проверяем поле 1 в Directory Entry (EntityNumber). Должно быть 124
	EntityNumber = ReadIntegerIn_D_Section(D_SectionVector, Entity124_D_Sect_Str_Number, 1);
	// stoi(D_SectionVector[Entity124_D_Sect_Str_Number - 1].substr(0, 8));     Удалить
	if (EntityNumber != 124) {
		string txt = "Ошибка в исходном файле!\nВ разделе Directory Entry неверное значение в поле 1 в строке " + to_string(Entity124_D_Sect_Str_Number) +
			".\nУказано значение " + to_string(EntityNumber) + " (должно быть 124).";
		PrintErrorAndExit(txt);
	}
	// Читаем поле 2 - Parameter Data Pointer. Проверяем корректность значения: должно быть больше 0, но меньше размера P_SectionVector.size()
	Field2_ParameterDataLine = ReadIntegerIn_D_Section(D_SectionVector, Entity124_D_Sect_Str_Number, 2);
	// stoi(D_SectionVector[Entity124_D_Sect_Str_Number - 1].substr(8, 8));    Удалить
	if (Field2_ParameterDataLine <= 0) {
		string txt = "Ошибка в исходном файле!\nВ разделе Directory Entry неверное значение в поле 2 элемента в строке " + to_string(Entity124_D_Sect_Str_Number) +
			".\nУказано значение " + to_string(Field2_ParameterDataLine) + " (значение должно быть больше нуля).";
		PrintErrorAndExit(txt);
	}
	else if (Field2_ParameterDataLine > P_SectionVector.size()) {
		string txt = "Ошибка в исходном файле!\nВ разделе Directory Entry неверное значение в поле 2 элемента в строке " + to_string(Entity124_D_Sect_Str_Number) +
			".\nУказано значение " + to_string(Field2_ParameterDataLine) + " \n(значение должно быть меньше размера Parameter Data Section).";
		PrintErrorAndExit(txt);
	}
	TmpEntity124_TrMatrix.D_Section_String_Number = Entity124_D_Sect_Str_Number;
	TmpEntity124_TrMatrix.P_Section_String_Number = Field2_ParameterDataLine;
	// Читаем поле 7 - Указатель на вложенную Transformation Matrix. Проверяем корректность значения.
	Field7_TrMatrixPointer = ReadIntegerIn_D_Section(D_SectionVector, Entity124_D_Sect_Str_Number, 7);
	// stoi(D_SectionVector[Entity124_D_Sect_Str_Number - 1].substr(48, 8));      Удалить
	if (Field7_TrMatrixPointer < 0) {
		string txt = "Ошибка в исходном файле!\nВ разделе Directory Entry неверное значение в поле 7 элемента в строке " + to_string(Entity124_D_Sect_Str_Number) +
			".\nУказано значение " + to_string(Field7_TrMatrixPointer) + " (значение должно быть больше или равно нулю).";
		PrintErrorAndExit(txt);
	}
	else if (Field7_TrMatrixPointer == Entity124_D_Sect_Str_Number) {
		string txt = "Ошибка в исходном файле!\nВ разделе Directory Entry неверное значение в поле 7 элемента в строке " + to_string(Entity124_D_Sect_Str_Number) +
			".\nУказано значение " + to_string(Field7_TrMatrixPointer) + " (значение не должно быть равно номеру собственной строки).";
		PrintErrorAndExit(txt);
	}
	else if (Field7_TrMatrixPointer > D_SectionVector.size()) {
		string txt = "Ошибка в исходном файле!\nВ разделе Directory Entry неверное значение в поле 7 элемента в строке " + to_string(Entity124_D_Sect_Str_Number) +
			".\nУказано значение " + to_string(Field7_TrMatrixPointer) + " \n(значение должно быть меньше размера Directory Entry Section).";
		PrintErrorAndExit(txt);
	}
	// Читаем поле 15 - Form Number. Проверяем корректность значения - должно быть 1 или 2.
	Field15_FormNumber = ReadIntegerIn_D_Section(D_SectionVector, Entity124_D_Sect_Str_Number, 15);
	if (Field15_FormNumber < 0 || Field15_FormNumber > 1) {   // Проверяем корректность значения - должно быть 1 или 2. Иначе - ошибка и выход из программы
		string txt = "Ошибка в исходном файле!\nВ разделе Directory Entry неверное значение в поле 15 элемента в строке " + to_string(Entity124_D_Sect_Str_Number) +
			".\nУказано значение " + to_string(Field15_FormNumber) + " (значение должно быть 0 или 1).";
		PrintErrorAndExit(txt);
	}
	if (Field15_FormNumber == 1) {
		string txt = "Внимание!\nВ IGES-файле обнаружена left-handed матрица трансформации.\n";
		txt = txt + "В Directory Entry Section, матрица трансформации Entity Type Number 124\nна строке " + to_string(Entity124_D_Sect_Str_Number) +
			", значение в поле 15 равно 1 (должно быть = 0).\nВозможен некорректный пересчёт координат.";
		PrintErrorAndExit(txt);
	}
	TmpEntity124_TrMatrix.FormNumber = Field15_FormNumber;
	// --- Читаем информацию о матрице трансформации в Parameter Data ---
	{
		int i = (Field2_ParameterDataLine - 1) * Global_PSectionLineLength;   // Индекс читаемого символа в Parameter Data Section
		// Читаем и проверяем первое поле в Parameter Data Section (Entity Type Number). Должно быть 124	
		EntityNumber = ReadIntegerDataType(P_SectionVector, i, 'P');
		if (EntityNumber != 124) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data неверное значение в первом поле в строке " + to_string(Field2_ParameterDataLine) +
				".\nУказано значение " + to_string(EntityNumber) + " (должно быть 124).";
			PrintErrorAndExit(txt);
		}
		// Читаем разделитель после первого поля в Parameter Data Section (т.е. после Entity Type Number)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после первого поля\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем второе значение в Parameter Data Section, т.е. аргумент индекс 1 (R11)
		TmpEntity124_TrMatrix.R11 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после второго поля - после R11\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем третье значение в Parameter Data Section, т.е. аргумент индекс 2 (R12)
		TmpEntity124_TrMatrix.R12 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после третьего поля - после R12\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем четвёртое значение в Parameter Data Section, т.е. аргумент индекс 3 (R13)
		TmpEntity124_TrMatrix.R13 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после четвёртого поля - после R13\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем пятое значение в Parameter Data Section, т.е. аргумент индекс 4 (T1)
		TmpEntity124_TrMatrix.T1 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после пятого поля - после T1\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем шестое значение в Parameter Data Section, т.е. аргумент индекс 5 (R21)
		TmpEntity124_TrMatrix.R21 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после шестого поля - после R21\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем седьмое значение в Parameter Data Section, т.е. аргумент индекс 6 (R22)
		TmpEntity124_TrMatrix.R22 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после седьмого поля - после R22\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем восьмое значение в Parameter Data Section, т.е. аргумент индекс 7 (R23)
		TmpEntity124_TrMatrix.R23 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после восьмого поля - после R23\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем девятое значение в Parameter Data Section, т.е. аргумент индекс 8 (T2)
		TmpEntity124_TrMatrix.T2 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после девятого поля - после T2\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем десятое значение в Parameter Data Section, т.е. аргумент индекс 9 (R31)
		TmpEntity124_TrMatrix.R31 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после деcятого поля - после R31\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем одиннадцатое значение в Parameter Data Section, т.е. аргумент индекс 10 (R32)
		TmpEntity124_TrMatrix.R32 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после одиннадцатого поля - после R32\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем двенадцатое значение в Parameter Data Section, т.е. аргумент индекс 11 (R33)
		TmpEntity124_TrMatrix.R33 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "Ошибка в исходном файле!\nВ разделе Parameter Data Section в строке " + to_string(Field2_ParameterDataLine) +
				" неверный разделитель после двенадцатого поля - после R33\n(должен быть символ  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// Читаем тринадцатое значение в Parameter Data Section, т.е. аргумент индекс 12 (T3)
		TmpEntity124_TrMatrix.T3 = ReadRealDataType(P_SectionVector, i, 'P');
	}
	TransformationMatrixVector.push_back(TmpEntity124_TrMatrix);   // 
	// Вызываем эту функцию рекурсивно если есть вложенная матрица
	if (Field7_TrMatrixPointer != 0) ReadEntity124_TrMatrix(TransformationMatrixVector, D_SectionVector, P_SectionVector, Field7_TrMatrixPointer);
}    //  Конец функции  ReadEntity124_TrMatrix()
//
// 
// Преобразует координаты точки Xorigin, Yorigin, Zorigin (заданы в IGES-файле) в координаты X1, Y1, Z1 используя матрицы 
// преобразования  TransformationMatrixVector. Единицы измерения сохраняются.
void TransformPointByTransfMatrix(const double& Xorigin, const double& Yorigin, const double& Zorigin, double& X1, double& Y1, double& Z1, vector<Entity124_TransformationMatrix>& TransformationMatrixVector)
{
	if (TransformationMatrixVector.empty()) {   //  Если матриц трансформации нет, то X1, Y1, Z1 равны Xorigin, Yorigin, Zorigin
		X1 = Xorigin;     Y1 = Yorigin;     Z1 = Zorigin;
		return;
	}
	for (const Entity124_TransformationMatrix& Entity124TrMatrxOBJ : TransformationMatrixVector) {
		X1 = Entity124TrMatrxOBJ.R11 * Xorigin + Entity124TrMatrxOBJ.R12 * Yorigin + Entity124TrMatrxOBJ.R13 * Zorigin + Entity124TrMatrxOBJ.T1;
		Y1 = Entity124TrMatrxOBJ.R21 * Xorigin + Entity124TrMatrxOBJ.R22 * Yorigin + Entity124TrMatrxOBJ.R23 * Zorigin + Entity124TrMatrxOBJ.T2;
		Z1 = Entity124TrMatrxOBJ.R31 * Xorigin + Entity124TrMatrxOBJ.R32 * Yorigin + Entity124TrMatrxOBJ.R33 * Zorigin + Entity124TrMatrxOBJ.T3;
	}
	return;
}




