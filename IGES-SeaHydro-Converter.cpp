//  ���������   IGES-SeaHydro_Converter. ������ 1.0.
//  ���������  IGES-SeaHydro_Converter.exe  ������������ 3D-��������� ����� ������� ����� �� ����� ���� IGES (���������� .igs)
//  � ������ ����� ���� � ��������� ������� ��� ���������  SeaHydro (������� �� ������������, ������������,
//  ��������������� �������; ����������� - ��� "�� ���" www.seatech.ru). 
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
// ���������� ��������� IGES-����� (�� ������� Global Parameter)
struct g_GParameters {
	char GP1_ParameterDelimiter;  // Global Parameter 1 - Parameter Delimiter (���������� �������� 1 - ����������� ����������). �� ��������� = ',' 
	char GP2_RecordDelimiter;  // Global Parameter 2 - Record Delimiter (���������� �������� 2 - ����������� �������). �� ��������� = ';' 
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
	int GP14_UnitsFlag; // Global Parameter 14 - Units Flag (��� ������� ���������). default = 1 (inches);  2 - millimeters;  6 - meters.
	string GP15_UnitsName; // Global Parameter 15 - Units Name. String naming the model units in the system; it shall be as field 14 unless field 14 is 3. The default value is 1. Postprocessors shall ignore this field if it is inconsistent with field 14.
	int GP16_MaxNumLineWeightGradations; // Global Parameter 16 - Maximum Number of Line Weight Gradations. The value shall be greater than zero. The default value is 1.
	double GP17_MaximumLineWidth; // Global Parameter 17 - Width of Maximum Line Weight in Units. No default value.
	string GP18_DateTimeFileGeneration; // Global Parameter 18 - Date and Time of Exchange File Generation. 15HYYYYMMDD:HHNNSS  or  13HYYMMDD:HHNNSS  .  No default value.
	double GP19_MinimumResolution; // Global Parameter 19 - Minimum User-Intended Resolution. No default value.
	double GP20_MaxCoordValue; // Global Parameter 20 - Approximate Maximum Coordinate Value. The default value is 0.0.
	string GP21_AuthorName; // Global Parameter 21 - Name of Author.  The default value is NULL, which is interpreted as �unspecified�.
	string GP22_AuthorsOrganization; // Global Parameter 22 - Author�s Organization. The default value is NULL, which is interpreted as �unspecified�.
	int GP23_VersionFlag; // Global Parameter 23 - Version Flag. The default value is 3. Value less than 1 shall assign 3; value greater than 11 shall assign 11.
	int GP24_DraftingStandardFlag; // Global Parameter 24 - Drafting Standard Flag.  The default value is 0.
	string GP25_DateTimeNativModelModified; // Global Parameter 25 - Date and Time Model was Created or Modified. The default value is NULL
	string GP26_AppProtocolIdentifier; // Global Parameter 26 - Application Protocol/Subset Identifier. The default value is NULL.  
};
//
struct Entity124_TransformationMatrix {  // ������� ������������� (Transformation Matrix Entity - Type 124)
	int D_Section_String_Number;  // ����� ������ � ������� �� ���� ������� � Directory Entry Section
	int P_Section_String_Number;  // ����� ������ � ������� �� ���� ������� � Parameter Data Section
	int FormNumber;  //  ���� 15 � Directory Entry. 0(default) - "����������" ������� ���������. 1 - "���������" ������� ���������.
	double R11, R12, R13, T1, R21, R22, R23, T2, R31, R32, R33, T3;
};
//
struct Entity110_Line {  // ���������� �� �������� "�������" (Entity Type 110 - Line). ����, ����������, ������� �������������.
	int D_Section_String_Number;  // ����� ������ � ������� �� ���� ������� � Directory Entry Section
	int P_Section_String_Number;  // ����� ������ � ������� �� ���� ������� � Parameter Data Section
	int Color;  // "����������" ����� 0-8. ������������� ���� - 10-������� ����� ������� 1RGB (�������� ��� RGB:254,001,220 ��� 1254001220).
	int FormNumber;  //  ���� 15 � Directory Entry. 0 - �������. 1 - ��� � ������� � (X1,Y1,Z1). 2 - ����������� ������
	double X1origin, Y1origin, Z1origin, X2origin, Y2origin, Z2origin;  // X1origin, Y1origin, Z1origin, X2origin, Y2origin, Z2origin - ���������� ������ � ����� ������� (��� �������� � IGES-�����, ��� ��������������)
	double X1, Y1, Z1, X2, Y2, Z2;  // - ���������� ������� � ������� ��������� 3D-������ (����� �������������� ����� ������� �������������, �.�. ����� ������� TransformPointByTransfMatrix())
	long long X1_01mm, Y1_01mm, Z1_01mm, X2_01mm, Y2_01mm, Z2_01mm;  // ���������� X1, Y1, Z1, X2, Y2, Z2 ����� ��������� �� 0,1��. ������� ��������� - 0,1�� (����� �����, ����� �������� ������ �������� � ����������)
	long long Length;  // ����� ������� ����� �������������� ����� ������� ������������� � ���������� �� 0,1��. ������� ��������� - 0,1�� (����� �����, ����� �������� ������ �������� � ����������)
	vector<Entity124_TransformationMatrix> TransformationMatrixVector; // ������ � ��������� ������������� (Transformation Matrix Entity - Type 124)
};
//
struct ShipTransformationMatrix {  // ������� ������������� � ������� ������� ��������� �� ��������� 3D-������. ���������� ����� ������� ��������� �������� ����.
	double R11, R12, R13, T1, R21, R22, R23, T2, R31, R32, R33, T3;
};
//
// ��������� ����� ����������� ������ � ����. Black = 0, Blue = 1, Green = 2, Cyan = 3, Red = 4, Magenta = 5, Brown = 6, LightGray = 7,
//  DarkGray = 8, LightBlue = 9, LightGreen = 10, LightCyan = 11, LightRed = 12, LightMagenta = 13, Yellow = 14, White = 15. 
//  ���������� ���� ������ - SetColor(7,0).
void SetColor(int TextColor, int bg = 0);
//
// ���� ����� ���� int � ���������
int getInt();
//
// ���� ����� ���� int � ���������: ������������ �������� ������ �� Min �� Max ������������.
int getInt_WithValueCheck(int Min, int Max);
//
//  �������� ��������� 
void PrintPreamble(const char* SourceFileName);
//
// �������� ������� �� ���������, ������ ��������� ���� help.txt � ������� �������
void PrintHelp(const char* HelpFileName = "Help.txt");
//
// ������� ������� ������� ENTER. ��� ��������� ������� ������������, ����� �� ����������
void PressENTER();
// 
//  �������� ������� ������� ��������� � ����� ������� ENTER ������� �� ��������� 
void PrintErrorAndExit(const string& Text);
//
// ��������� �������� �� ������������ ����������� � �������� ��� Global Parameter 1 � Global Parameter 2 (�� IGES Specification Version 6.0, 2.2.3.1).
// ���������� true  ���� ������ �������������, �����- false.
bool CheckGP1GP2(const char& ch);
//
// ������ � ��������� g_GParameters ���������� ��������� �� G_SectionVector, ��������� �� �� IGES Specification V.6.0. 
// ��������� ���������� � ���������� ���������� � InfoFileText. 
void ReadAndCheckGlobalParameters(const vector<string>& G_SectionVector, string& InfoFileText);
//
// ������ ������ ����������� (Parameter Delimiter  ���  Record Delimiter) ������� � ������� � �������� i � �������� Global 
// Section  �  Parameter Data Section. ������������� �������� i �� ������, ��������� �� ������������ ��� �� ������������
// ��������� (���� ��� ����). ��������  Section  ���������� ������ ('G'- Global Section, 'P'- Parameter Data Section).
// ���������� ����������� ������.
char ReadDelimiters(const vector<string>& SectionVector, int& i, const char& Section);
//
// ������ � ���������� �������� ���� String, ������� ������ � ������� i � �������� Global Section  �  Parameter Data
// Section. ������ �������� ������� i �� ������, ��������� �� ����������� �������. ��������  Section  ���������� ������,
// � ������� ���������� ������ ('G'- Global Section, 'P'- Parameter Data Section).
string Read_String(const vector<string>& SectionVector, const char& GP1_ParameterDelimiter, const char& GP2_RecordDelimiter, int& i, const char& Section);
//
// ������ � ���������� ����� ����� ������� ������ � ������� i � �������� Global Section  �  Parameter Data Section.
//  ������ �������� ������� i �� ������, ��������� �� ����������� �������. ��������  Section  ���������� ������,
//  � ������� ���������� ������ ('G'- Global Section, 'P'- Parameter Data Section).
long long  ReadIntegerDataType(const vector<string>& SectionVector, int& i, const char& Section);
//
// ������ � ���������� ����� � ��������� ������ ������� ������ � ������� i � �������� Global Section  �  Parameter Data 
// Section. ������ �������� ������� i �� ������, ��������� �� ����������� �������. ��������  Section  ���������� ������, 
// � ������� ���������� ������ ('G'- Global Section, 'P'- Parameter Data Section).
double ReadRealDataType(const vector<string>& SectionVector, int& i, const char& Section);
//
// ������ � ���������� ����� ����� � ������� Directory Entry ��� ������� �� ������ D_Sect_Str_Number � ���� FieldNumber.
// ���� ���� ��������� ��������� - ���������� 0. ���� � ���� ��������� ����� ������ ������ ����� - ������ � ����� �� ���������
int ReadIntegerIn_D_Section(const vector<string>& DSectionVector, const int D_Sect_Str_Number, const int FieldNumber);
//
// ������ ���� (�.�. �������������� ���� 13 - Color Number) ��� ������ Entity �� ������ Entity_D_Sect_Str_Number. 
// ����������: "����������" ���� (0-8), ��� ������������� ���� - 10-������� ����� ������� 1RGB (�������� ���  
// RGB:254,001,220 ��� 1254001220). "����������": 0-no color(default), 1-Black(RGB:0,0,0), 2-Red (RGB:255,0,0), 3-Green  
// (RGB:0,255,0), 4-Blue(RGB:0,0,255), 5-Yellow(RGB:255,255,0), 6-Magenta (RGB:255,0,255), 7-Cyan (RGB:0,255,255), 8-White (RGB:255,255,255).
int ReadEntityColor(const vector<string>& D_SectionVector, const vector<string>& P_SectionVector, const int D_Sect_Str_Number);
//
// ������ "�������" (Line Entity - Type 110) � Directory Entry Section �� ������ D_Sect_Str_Number.
// ������������ � ���������� ���������� � ������ Global_Entity110_Line_Vector. 
void ReadEntity110_Line(const vector<string>& D_SectionVector, const vector<string>& P_SectionVector, const int D_Sect_Str_Number);
// 
// ������ ������� ������������� (Transformation Matrix Entity - Type 124) � Directory Entry Section, �������
// ��������� �� ������ Entity124_D_Sect_Str_Number, � ����� ������ ��� �������� � �� ������� (���� ����� ����). ���������� ��������� � TransformationMatrixVector.
void ReadEntity124_TrMatrix(vector<Entity124_TransformationMatrix>& TransformationMatrixVector, const vector<string>& D_SectionVector,
	const vector<string>& P_SectionVector, int Entity124_D_Sect_Str_Number);
//
// ����������� ���������� ����� Xorigin, Yorigin, Zorigin (������ � IGES-�����) � ���������� X1, Y1, Z1 ��������� ������� 
// ��������������  TransformationMatrixVector. ������� ��������� �����������.
void TransformPointByTransfMatrix(const double& Xorigin, const double& Yorigin, const double& Zorigin, double& X1, double& Y1, double& Z1, vector<Entity124_TransformationMatrix>& TransformationMatrixVector);
//
const int Global_MaxSignifDigits{ 15 };  // ������������ ���������� �������� ����, ������� �������� ���������� � double ��� ������ ��������
const int Global_GSectionLineLength{ 72 };  // ����� ������ � ����������� � Global Section
const int Global_PSectionLineLength{ 64 };  // ����� ������ � ����������� � Parameter Data Section
const int Global_D_Section_Field_Size{ 8 };  // ���������� �������� ���������� ����� ����� � Directory Entry Section
const int Global_IGES_File_StringLength{ 80 };  // ����� ��������� ������ � IGES-�����
const int Global_Entity110LineNumber = 110;  // ����� ������� "�������" (Line Entity - Type 110)
const int Global_Entity314ColorNumber = 314;  // ����� ������� "����" (Color Definition Entity - Type 314)
const int Global_Entity124TransMatrixNumber = 124;  // ����� ������� "������� �������������" (Transformation Matrix Entity - Type 124)
g_GParameters Global_GlobalParameters;   // ���������� ��������� IGES-����� (�� ������� Global Parameter)
vector<Entity110_Line> Global_Entity110_Line_Vector;  // ������ �������� Entity110_Line - ������ � ����������� � ���� �������� (�������� Entity Type 110 (Line))
// 
//
//
int main() {
	setlocale(LC_ALL, "Russian");
	const int IGESfileStringLength{ 80 };  // ����� ������ ������ � IGES-�����
	const int TmpTxtLength{ 82 };  //  ����� ��������� ������ ��� ������ ����� �� IGES-�����
	const char IGESFileName[]{ "geom.igs" };  // ��� IGES-����� �������� ������ 
	const char InfoFileName[]{ "info.txt" };  // ��� �����, ���� � ������� ���� ������������ �������������� ���������� �� IGES-�����
	const char Entity110_Line_FileName[]{ "Entity110_Line.txt" };   // ��� �����, ���� ����� �������� ���������� �� �������� Entity110 "Line" ("�������")
	int Choice{ 0 };  // ����� ���������� ��������
	char TmpTxt[TmpTxtLength];
	string InfoFileText;  // ����� ��� ������ � ���� "info.txt"
	string Entity110_Line_FileText;  // ����� ��� ������ � ���� "Entity110_Line.txt"
	FILE* IGESFilePointer{ nullptr }; // ��������� �� �������� IGES-���� �������� ������ "geom.igs"
	FILE* InfoFilePointer{ nullptr }; // ��������� �� ���� "info.txt" 
	FILE* Entity110_Line_FilePointer{ nullptr }; // ��������� �� ���� "Entity110_Line.txt" � ����������� �� �������� Entity110 "Line" ("�������")
	//  
	vector<string>TmpInputVector;   //  ��������� ������ ��� �������������� ������ ����� ����������� IGES-�����
	//  ������� ��� ������ ����� �� �������� IGES-�����
	vector<string> S_SectionVector;  // ������ ��� ������ ����� �� ������� ������� - Start Section
	vector<string> G_SectionVector;  // ������ ��� ������ ����� �� ������� ������� - Global Section
	vector<string> D_SectionVector;  // ������ ��� ������ ����� �� �������� ������� - Directory Entry Section
	vector<string> P_SectionVector;  // ������ ��� ������ ����� �� ��������� ������� - Parameter Data Section
	vector<string> T_SectionVector;  // ������ ��� ������ ������ �� ���������� ������� - ������ Terminate Section
	//
	//
	//  �������� ��������� � ������� ����
	while (true) {
		PrintPreamble(IGESFileName);  //  �������� ���������
		//
		cout << "������� ����� ��� ����������� ��������:" << endl;
		SetColor(15);   cout << "0"; SetColor(7); cout << " - ���������� ���������� ���������; " << endl;
		cout << "1 - ������� �� ���������;" << endl;
		cout << "2 - ����� �� ���������." << endl;
		cout << " ��� �����: ";
		SetColor(13);     Choice = getInt_WithValueCheck(0, 2);     SetColor(7);
		if (Choice == 0) {  //  0 - ���������� ���������� ���������
			break;
		}
		else if (Choice == 1) {  //  1 - ������� �� ���������
			PrintHelp();  // �������� ������� �� ��������� � ������ ��������� ���� � ������� �������
		}
		else if (Choice == 2) {  //  2 - ����� �� ���������
			exit(0);
		}
		system("cls");
	}
	//  ���� ������ 0 - �� ���������� ���������� ���������
	while (true) {  // ��������� IGES-����. ����� �� ����� ������ ����� ��������� �������� �����
		system("cls");
		cout << "\n   ������ ������ �� IGES-�����  " << IGESFileName << " :" << endl;
		_set_errno(0);
		if (fopen_s(&IGESFilePointer, IGESFileName, "r")) {  //  ���� ������ ��� �������� �����
			SetColor(12);  cout << "\n  ������ ��� �������� ����� ";  SetColor(15);  cout << IGESFileName << endl;  SetColor(7);
			if (errno == ENOENT) {  // ���� ���� �����������
				SetColor(12);   cout << "  ���� � ����� ������ � ������� ����� �� ������\n  (�������� �������� - ��� ������� ����� ����� ������ ���� ����������)." << endl;   SetColor(7);
			}
			cout << "\n\n������� ����� ��� ����������� ��������:" << endl;
			SetColor(15);   cout << "0"; SetColor(7); cout << " - ��������� ������ �� �����  " << IGESFileName << " ;" << endl;
			cout << "1 - ����� �� ���������." << endl;
			cout << " ��� �����: ";
			SetColor(13);     Choice = getInt_WithValueCheck(0, 1);     SetColor(7);
			if (Choice == 0) {  // 0 - ��������� ������ �� �����
				continue;
			}
			else if (Choice == 1) {   //  1 - ����� �� ���������
				exit(0);
			}
		}
		else  break;     //  ���� ���� ������ ������� - �� ����� �� ����� � ����������� ���������
	}
	SetColor(10);  cout << "\n   ���� ";  SetColor(15);  cout << IGESFileName;  SetColor(10); cout << " ������ �������." << endl << endl;  SetColor(7);
	// ������ ���������� IGES-����� �� ��������� ������ �����  TmpInputVector
	while (fgets(TmpTxt, TmpTxtLength, IGESFilePointer) != NULL) { // �������� � TmpTxt ������, ������� ������ ����� ������ \n
		TmpTxt[IGESfileStringLength] = '\0';
		TmpInputVector.push_back(TmpTxt);
	}
	_fcloseall();
	// �������� ���������� TmpInputVector � ������� �� ��������
	{  // ��������� ���� - ����� ���������� � ����� ����� ����������  i
		int i = 0;  // ������ �������� � TmpInputVector (�.�. ������ �������������� ������)
		// �������� �� TmpInputVector ������ ������ (Start Section) � ������ S_SectionVector
		if (TmpInputVector[i][72] == 'S')
		{
			while (TmpInputVector[i][72] == 'S') {
				S_SectionVector.push_back(TmpInputVector[i]);
				i++;
			}
		}
		else { PrintErrorAndExit("   ������! �� ������ ������ Start Section"); }
		// �������� �� TmpInputVector ������ ������ (Global Section) � ������ G_SectionVector
		if (TmpInputVector[i][72] == 'G')
		{
			while (TmpInputVector[i][72] == 'G') {
				G_SectionVector.push_back(TmpInputVector[i]);
				i++;
			}
		}
		else { PrintErrorAndExit("   ������! �� ������ ������ Global Section"); }
		// �������� �� TmpInputVector ������ ������ (Directory Entry Section) � ������ D_SectionVector
		if (TmpInputVector[i][72] == 'D')
		{
			while (TmpInputVector[i][72] == 'D') {
				D_SectionVector.push_back(TmpInputVector[i]);
				i++;
			}
		}
		else { PrintErrorAndExit("   ������! �� ������ ������ Directory Entry Section"); }
		// �������� �� TmpInputVector �������� ������ (Parameter Data Section) � ������ P_SectionVector
		if (TmpInputVector[i][72] == 'P')
		{
			while (TmpInputVector[i][72] == 'P') {
				P_SectionVector.push_back(TmpInputVector[i]);
				i++;
			}
		}
		else { PrintErrorAndExit("   ������! �� ������ ������ Parameter Data Section"); }
		// �������� �� TmpInputVector ��������� ������ (������ Terminate Section) � ������ T_SectionVector
		if (TmpInputVector[i][72] == 'T') T_SectionVector.push_back(TmpInputVector[i]);
		else { PrintErrorAndExit("   ������! �� ������ ������ Terminate Section"); }
	}
	// �������� ����������� ������
	{
		int S, G, D, P;  // �������� ����� � ������� Terminate Section
		// �������� ������� Directory Entry Section: ���������� ����� ������ ���� ������
		if (D_SectionVector.size() % 2 != 0)
			PrintErrorAndExit("   ������! � ������� Directory Entry Section �������� ���������� �����.");
		//  ������ ���� S ������� Terminate Section
		if (T_SectionVector[0][0] == 'S')   S = stoi(T_SectionVector[0].substr(1, 7));
		else {
			PrintErrorAndExit("   ������! �� ������� ��������� ���� S � ������� Terminate Section.");
		}
		//  ������ ���� G ������� Terminate Section
		if (T_SectionVector[0][8] == 'G')   G = stoi(T_SectionVector[0].substr(9, 7));
		else {
			PrintErrorAndExit("   ������! �� ������� ��������� ���� G � ������� Terminate Section.");
		}
		//  ������ ���� D ������� Terminate Section
		if (T_SectionVector[0][16] == 'D')   D = stoi(T_SectionVector[0].substr(17, 7));
		else {
			PrintErrorAndExit("   ������! �� ������� ��������� ���� D � ������� Terminate Section.");
		}
		//  ������ ���� P ������� Terminate Section
		if (T_SectionVector[0][24] == 'P')   P = stoi(T_SectionVector[0].substr(25, 7));
		else {
			PrintErrorAndExit("   ������! �� ������� ��������� ���� P � ������� Terminate Section.");
		}
	}
	//  ������ � InfoFileText ������ ������ (Start Section) �� ������� S_SectionVector
	InfoFileText += "    �������������� ������ �� �����  \"";
	InfoFileText += IGESFileName;
	InfoFileText += "\"\n    -----------------------------------------------\n\n";
	InfoFileText += "    Start Section:\n";
	InfoFileText += "    --------------\n";
	for (string& elem : S_SectionVector) {
		InfoFileText += elem.substr(0, 72);
		InfoFileText += '\n';
	}
	InfoFileText += '\n';
	// ������ � InfoFileText ���������� ��������� �� ������� ������� (Global Section) 
	InfoFileText = InfoFileText + "\n    Global Section:\n" + "    ----------------\n";  // ����� ��� ���������� ������ � ����
	// ������ ������ Global Section
	ReadAndCheckGlobalParameters(G_SectionVector, InfoFileText);
	// ������ ������� �� ������� Directory Entry Section
	{
		int D_Sect_Str_Number{ 1 }; // ����� ������ ���������� Entity � Directory Entry
		int EntityNumber{ 0 }; // ����� (Entity Type Number) ��������, ������� �������� � ��������� ������
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

	// �������
	cout << "\n____���������� � ����������� �������� Entity110_Line____" << endl;
	for (Entity110_Line Entity110LineOBJ : Global_Entity110_Line_Vector) {
		cout << "������ 110: ������ �" << Entity110LineOBJ.D_Section_String_Number << " � Directory Entry,  ������ �" << Entity110LineOBJ.P_Section_String_Number << " � Parameter Data." << endl;
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
	// �������    */


	// �������� �� ����� �������� ���������� � ����������� ��������� (�������, ������� ���������)
	cout << "\n      �������� ���������� � ����������� ���������" << endl;
	cout << "   -------------------------------------------------------" << endl;
	//cout << "������� ������������ ������:  ";
	printf("   ������� ������������ ������:  %e\n", Global_GlobalParameters.GP13_ModelSpaceScale);
	cout << "   ������� ���������:  ";
	if (Global_GlobalParameters.GP14_UnitsFlag == 2) cout << "�� (����������)" << endl;
	else if (Global_GlobalParameters.GP14_UnitsFlag == 6) cout << "� (�����)" << endl;
	cout << "   -------------------------------------------------------" << endl;
	cout << "\n\n\n";

	//  ������ ���������� �� ������ InfoFileText  � ���� "info.txt" 
	cout << "   -------------------------------------------------------" << endl;
	cout << "   ����� ��������� ������� ����   "; SetColor(15);  cout << InfoFileName; SetColor(7);  cout << "  � �������������� " << endl;
	cout << "   ����������� �� IGES-�����  ";  SetColor(15);  cout << IGESFileName; SetColor(7);  cout << " .  ���� ���� ";
	SetColor(15); cout << InfoFileName << endl; SetColor(7); cout << "   ��� ����������, �� �� ����� �����, � ������ ���� " << endl;
	cout << "   ����� ������ ����� ����." << endl << endl;
	cout << "      ������� ENTER ��� �����������" << endl;
	PressENTER();
	_set_errno(0);
	if (fopen_s(&InfoFilePointer, InfoFileName, "w")) {  //  ���� ������ ��� �������� � �������� �����
		_set_errno(0);
		PrintErrorAndExit(string("\n  ������ ��� �������� �����  ") + string(InfoFileName));
	}
	else {   //  ���� ���� ������� ������ � ������ ��� ������, ��
		if (fputs(InfoFileText.c_str(), InfoFilePointer) >= 0) {  //  ���������� ����� � ����       
			SetColor(10);  cout << "\n����  "; SetColor(15);  cout << InfoFileName;  SetColor(10);  cout << "  ������� �������." << endl; SetColor(7);
			_fcloseall();
			cout << "\n������� ENTER ��� �����������" << endl;
			PressENTER();
		}
		else {
			SetColor(12);   cout << "\n  ������ ��� ������ � ���� ";   SetColor(15);   cout << InfoFileName;  SetColor(7);
			_fcloseall();
			cout << "\n������� ENTER ��� �����������" << endl;
			PressENTER();
		}
	}
	_fcloseall();

	// --- ������ ���������� � ���� "Entity110_Line.txt" 
	cout << "\n\n   -------------------------------------------------------" << endl;
	cout << "   ����� ��������� ������� ����  "; SetColor(15);  cout << Entity110_Line_FileName; SetColor(7);  cout << "  � �����������  " << endl;
	cout << "   �� �������� Entity110 \"Line\" (\"�������\").  ���� ����  "; SetColor(15); cout << Entity110_Line_FileName << endl; SetColor(7);
	cout << "   ��� ����������, �� �� ����� �����, � ������ ���� ����� ������ ����� ����." << endl << endl;
	cout << "      ������� ENTER ��� �����������" << endl;
	PressENTER();
	// ��������� ����� �� �������� Entity110 "Line" ("�������") ��� ������ � ���� "Entity110_Line.txt" 
	Entity110_Line_FileText += "Entity_Type X1origin Y1origin Z1origin X2origin Y2origin Z2origin X1 Y1 Z1 X2 Y2 Z2 Length Color FormNumber\n";
	for (Entity110_Line Entity110LineOBJ : Global_Entity110_Line_Vector) {
		char X1originTXT[_CVTBUFSIZE], Y1originTXT[_CVTBUFSIZE], Z1originTXT[_CVTBUFSIZE], X2originTXT[_CVTBUFSIZE], Y2originTXT[_CVTBUFSIZE], Z2originTXT[_CVTBUFSIZE],
			X1_TXT[_CVTBUFSIZE], Y1_TXT[_CVTBUFSIZE], Z1_TXT[_CVTBUFSIZE], X2_TXT[_CVTBUFSIZE], Y2_TXT[_CVTBUFSIZE], Z2_TXT[_CVTBUFSIZE], LengthTXT[_CVTBUFSIZE];
		if (_gcvt_s(X1originTXT, _CVTBUFSIZE, Entity110LineOBJ.X1origin, Global_MaxSignifDigits) ||    // ������������ double � string. ���� ������ ��� ���������������, �� ����� �� ���������
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
			PrintErrorAndExit("������ ��� ������ _gcvt_s() ��� ������������ Entity110_Line_FileText.");
		}
		Entity110_Line_FileText = Entity110_Line_FileText + "Entity100_\"Line\" " + X1originTXT + " " + Y1originTXT + " " + Z1originTXT + " " +
			X2originTXT + " " + Y2originTXT + " " + Z2originTXT + " " + X1_TXT + " " + Y1_TXT + " " + Z1_TXT + " " +
			X2_TXT + " " + Y2_TXT + " " + Z2_TXT + " " + LengthTXT + " " + to_string(Entity110LineOBJ.Color) + " " + to_string(Entity110LineOBJ.FormNumber) + " \n";
	}
	// ������ � ���� "Entity110_Line.txt"
	_set_errno(0);
	if (fopen_s(&Entity110_Line_FilePointer, Entity110_Line_FileName, "w")) {  //  ���� ������ ��� �������� � �������� �����
		_set_errno(0);
		PrintErrorAndExit(string("\n  ������ ��� �������� �����  ") + string(Entity110_Line_FileName));
	}
	else {   //  ���� ���� ������� ������ � ������ ��� ������, ��
		if (fputs(Entity110_Line_FileText.c_str(), Entity110_Line_FilePointer) >= 0) {  //  ���������� ����� � ����       
			SetColor(10);  cout << "\n����  "; SetColor(15);  cout << Entity110_Line_FileName;  SetColor(10);  cout << "  ������� �������." << endl; SetColor(7);
			_fcloseall();
			cout << "\n������� ENTER ��� �����������" << endl;
			PressENTER();
		}
		else {
			SetColor(12);   cout << "\n  ������ ��� ������ � ���� ";   SetColor(15);   cout << Entity110_Line_FileName;  SetColor(7);
			_fcloseall();
			cout << "\n������� ENTER ��� �����������" << endl;
			PressENTER();
		}
	}
	_fcloseall();
	// --- ����������� ������� ������������� ��� ��������� �� ��������� 3D-������ � ������� ������� ��������� ---
	{
		// ����� � Global_Entity110_Line_Vector �������� ���� - 3 ����� ������� ������� 
		int AxisX_index{ 0 }, AxisY_index{ 0 }, AxisZ_index{ 0 };  // ������ ��������� �������� ���� � ������� Global_Entity110_Line_Vector. AxisX_index - ������ ������ �������� �������. AxisY_index - ������ ������� �� ����� �������. AxisZ_index - ������ �������� �� ����� �������
		for (int i = 0; i < Global_Entity110_Line_Vector.size(); i++) {
			if (Global_Entity110_Line_Vector[i].Length > Global_Entity110_Line_Vector[AxisX_index].Length) 	AxisX_index = i;
			else if (Global_Entity110_Line_Vector[i].Length > Global_Entity110_Line_Vector[AxisY_index].Length)	AxisY_index = i;
			else if (Global_Entity110_Line_Vector[i].Length > Global_Entity110_Line_Vector[AxisZ_index].Length)	AxisZ_index = i;
		}
		cout << "\n\nAxisX_index=" << AxisX_index << ";   AxisY_index=" << AxisY_index << ";   AxisZ_index=" << AxisZ_index << endl << endl;  //  �������
		// �������� ��������� �������� ����. ��� ������ ���� � ����� ����������, ����������� ���� ��������� 3D-������ � ������� ��������������� ���� �����.
		for (int i = 0; i < Global_Entity110_Line_Vector.size(); i++) {   //  ��������, ��� ������� "��� X" (����� ������� �������) - � ������������ ����������
			if (i == AxisX_index) continue;
			if (Global_Entity110_Line_Vector[i].Length == (Global_Entity110_Line_Vector[AxisX_index].Length)) { // ���� ������ ������ ������� "��� X" - ������ � �����
				string txt = "������ � �������� �����!\n������ ������ ������� \'��� X\' (����� ������� ������� � 3D-������).\n";
				txt = txt + "������� \'��� X\' ������ ���� ������ ����.\n  ���������� � ��������� ��������:\n" +
					"O������ 1: X1=" + to_string(Global_Entity110_Line_Vector[AxisX_index].X1) + "; Y1=" + to_string(Global_Entity110_Line_Vector[AxisX_index].Y1) +
					"; Z1=" + to_string(Global_Entity110_Line_Vector[AxisX_index].Z1) + ";      X2=" + to_string(Global_Entity110_Line_Vector[AxisX_index].X2) +
					"; Y2=" + to_string(Global_Entity110_Line_Vector[AxisX_index].Y2) + "; Z2=" + to_string(Global_Entity110_Line_Vector[AxisX_index].Z2) +
					".\n O������ 2: X1=" + to_string(Global_Entity110_Line_Vector[i].X1) + "; Y1=" + to_string(Global_Entity110_Line_Vector[i].Y1) +
					"; Z1=" + to_string(Global_Entity110_Line_Vector[i].Z1) + ";      X2=" + to_string(Global_Entity110_Line_Vector[i].X2) +
					"; Y2=" + to_string(Global_Entity110_Line_Vector[i].Y2) + "; Z2=" + to_string(Global_Entity110_Line_Vector[i].Z2) + ".\n";
				PrintErrorAndExit(txt);
			}
		}

	}




	// ���������� ���������
	SetColor(10);    cout << "       ��������� ��������� �������." << endl << endl;    SetColor(7);
	cout << "    ������� ENTER ��� ������ �� ���������" << endl;
	PressENTER();
}   //   ����� ������� main()
//
// 
//  �������� ��������� 
void PrintPreamble(const char* SourceFileName)
{
	system("cls");
	cout << "  ---------------------------------------------------------------------------  " << endl;
	cout << "  ---------------    ���������   IGES-SeaHydro_Converter    -----------------  " << endl;
	cout << "  ---------------                 ������ 1.0.               -----------------  " << endl;
	cout << "  ---------------------------------------------------------------------------  " << endl;
	cout << endl;
	cout << "  ���������  \"IGES-SeaHydro_Converter\" ������ 3D-��������� �����" << endl;
	cout << "  ������� ����� �� ����� ���� IGES (����������.igs) � ������ ����� ����" << endl;
	cout << "  � ��������� ������� ��� ���������  \"SeaHydro\"." << endl;
	cout << endl;
	cout << "  ��������� ������ ���������� � ��������� ����� ����� �� IGES-�����:" << endl;
	SetColor(12); cout << "  - \"Circular Arc\" (Type Number 100) (_���_�_������_������_);" << endl;  SetColor(7);
	SetColor(12); cout << "  - \"Conic Arc\" (Type Number 104) (_���_�_������_������_);" << endl;  SetColor(7);
	SetColor(10);  cout << "  - \"Line\" (Type Number 110);" << endl;  SetColor(7);
	SetColor(12); cout << "  - \"Parametric Spline Curve\" (Type Number 112) (_���_�_������_������_);" << endl;  SetColor(7);
	SetColor(12); cout << "  - \"Rational B-Spline Curve\" (Type Number 126) (_���_�_������_������_);" << endl;  SetColor(7);
	cout << endl;
	cout << "���������� � IGES-����� ������� � �������." << endl;
	cout << endl;
}
//
//
// �������� ������� �� ���������, ������ ��������� ���� help.txt � ������� �������
void PrintHelp(const char* HelpFileName)
{
	int Choice{ 0 };  // ����� ���������� ��������
	FILE* HelpFilePointer{ nullptr }; // ��������� �� ���� 
	string heading = { "  ---------------------------------------------------------------------------  \n\
           ������� �� ���������   \"IGES-SeaHydro_Converter\"  ������ 1.0.    \n\
  ---------------------------------------------------------------------------  \n"
	};
	// ����� �������
	char txt[] = { "\
1. �������� ���������.                                                        \n\
  ��������� IGES-SeaHydro_Converter.exe ������ IGES-���� (���������� .igs)    \n\
  c 3D-���������� ����� ������� ����� � ������ ����� ���� � ������������     \n\
  ���������� ����� ��� ������������� � �������� ������������ � ���������������\n\
  � ��������� \"SeaHydro\" (����������� ��������� \"SeaHydro\"- ����� ��� \"�� ���\",\n\
  ���� www.seatech.ru).                                                       \n\
\n\
  ��������� ������ ��������� ���� ����� (curve entities) �� IGES-�����:       \n\
         - \"Circular Arc\" (Type Number 100) (_���_�_������_������_);        \n\
         - \"Conic Arc\" (Type Number 104) (_���_�_������_������_);           \n\
  - \"Line\" (Type Number 110);                                               \n\
         - \"Parametric Spline Curve\" (Type Number 112) (_���_�_������_������_);\n\
         - \"Rational B-Spline Curve\" (Type Number 126) (_���_�_������_������_).\n\
  ������ ������� � IGES-����� (�����������, ����, ����� � ��.) ������������ � \n\
  �� ������ �� ������.                                                        \n\
\n\
2. ���������� � ��������� IGES-����� 3D-������.                               \n\
 2.1 ����� ���������� � �����:                                                \n\
   - ��� ����� - \"geom.igs\" (��� ����� ���������). ���� IGES-���� ����������\n\
     ����� ������ ��� - �� ������������ ��� � \"geom.igs\";                   \n\
   - ���� \"geom.igs\" ������ ���� � ����� ����� � exe-������ ���������.      \n\
 2.2 ���������� � ���������:                                                  \n\
  2.2.1 ����� ���������� � ���������:                                         \n\
   - ������� ��������� � ��������� - ���������� (��) ��� ����� (�).           \n\
   - ������� ������������ ������ = 1.0;                                       \n\
   - �������� ����� (����, �������, ����, ��� �����) �� ����� ��������;       \n\
   - ���������� ����� ����� ��������� � ��������� 0,1 ��.                     \n\
  2.2.2 �����, �������� ����������, ������� �� ��� ������:                    \n\
   - ������� ���� X, Y, Z - ��� ������� ���������������� ������� (������� Type\n\
     Number 110 \"Line\"), ��������� �� ����� �����. ���������� ������� �������\n\
     ��������� ���������� �� ������� ��������� 3D-������ � IGES-�����;        \n\
   - ������� ������ - ����������� ������� ��������, ����������� ��� X �       \n\
     ��������� �� ���� ��� X. ����� �������� ������� ������� �������� ���������\n\
     ���������. ����� ������� ����� ����� ��������� ������ ������� ���������; \n\
   - ����� ���������� - �������, ����, ������� � �.�., ��������� � �.1 �������,\n\
     ������� � ���������� ����������. ��������� ��������� ������� ���������.  \n\
  2.2.3 ���������� � �������� ����:                                           \n\
   - ������� ���� ������ ���� ��������� ���� \"Line\" (Entity Type Number 110);\n\
   - ������� ���� ������ ���� ������ �������� ��������� � 3D-������. �������  \n\
     \'��� X\' - ����� ������� �������, ������� \'��� Y\' ������ ��� X, �������\n\
     \'��� Z\' ������ ��� Y. ��� ��������� ������� � 3D-������ ������ ���� ������;\n\
   - ������� ���� ������ ���������� � ����� �����, ���� ������� ���������������,\n\
     ������ ������� ������ ���� ���������� �����-���� ��� ��������� 3D-������.\n\
     ����� ����������� �������� ���� ����� �������� ������� ���������� �������\n\
     ������� ��������� ��������������� �����;                                 \n\
   - ������� ���� �������� ����� ������� ���������: ������� \'��� X\' ���������\n\
     � ��� �����, ������� \'��� Y\' - �� ������ ����, ������� \'��� Z\' - �����.\n\
     ������� X-Y ���������� �������� ��������� (��), ������� X-Z ����������   \n\
     ������������� ��������� (��), ������� Y-Z - ��������� ������-���������.  \n\
  2.2.4 ���������� � �������� ������:                                         \n\
   - ������� ������ ������ ���� ��������� ���� \"Line\" (Entity Type Number 110);\n\
   - ������� ������ ������ ������ �������� ��� ������� � ��������� ��         \n\
     ����������� ��� X ���� ����� ������ �������� � �����.                    \n\
   - ����� ������� ������ ����� ����� ��������� ������ ������� ���������.     \n\
  2.2.5 ���������� � ������ ����������:                                       \n\
   - ������ ������� ��������� ������ ���� ��������� �������� ����� ���������� \n\
     � ������ � ���������, ���������� ����� �������� ������� ������.          \n\
   - � ������ ������ ��������� (������ 1.0.) ����� ���������� ������ ����     \n\
     ��������� ����:                                                          \n\
      - \"Line\" (Type Number 110).                                           \n\
                                                                              \n\
3. ��������� ��� IGES-�����:                                                  \n\
  - ������ \"��������� IGES-�����\":                                          \n\
       (����  https://unril.wordpress.com/2011/04/01/structure-iges-file/ );  \n\
  - \"The Initial Graphics Exchange Specification(IGES) Version 6.0\"           \n\
       (����  https://www.filemonger.com/specs/igs/devdept.com/version6.pdf );\n\
  - �����  ����� �� \"������ ���� (CAD/CAM/CAE)\".-���.: �����, 2004 .          \n\
"
	};
	system("cls");
	cout << endl;
	cout << heading; // ������ ��������� �������
	cout << "������� ����� ��� ����������� ��������:" << endl;
	cout << "0 - �������� ������� �� ������;" << endl;
	cout << "1 - ������� ���� "; SetColor(15); cout << HelpFileName; SetColor(7); cout << " � ������� �������; " << endl;
	cout << "2 - ����� �� �������." << endl;
	cout << " ��� �����: ";
	SetColor(13);     Choice = getInt_WithValueCheck(0, 2);     SetColor(7);
	// 
	if (Choice == 0) {  // 0 - ���������� ������� �� ������
		system("cls");
		cout << heading; // ������ ��������� �������
		cout << txt;
		cout << "\n������� ENTER ��� ������ �� �������" << endl;
		while (true) {
			if (_getch() == 13) break;
		}
	}
	else if (Choice == 1) {   // 1 - ������� ����
		cout << endl << endl;
		cout << "-------------------------------------------------------------------------------" << endl;
		cout << "��������� ������� ���� "; SetColor(15); cout << HelpFileName; SetColor(7); cout << ". ���� ���� � ����� ������ ��� ����������, �� ��" << endl;
		cout << "����� �����, � ������ ���� ����� ������ ����� ����." << endl;
		cout << "������� ����� ��� ����������� ��������:" << endl;
		SetColor(15);  cout << "0";  SetColor(7);  cout << " - ������� ���� ������� \"";  cout << HelpFileName;  cout << "\" ." << endl;
		cout << "1 - ����� �� �������." << endl;
		cout << " ��� �����: ";
		SetColor(13);     Choice = getInt_WithValueCheck(0, 1);     SetColor(7);
		if (Choice == 0) {   //  0 - ������� ���� �������
			_set_errno(0);
			if (fopen_s(&HelpFilePointer, HelpFileName, "w")) {  //  ���� ������ ��� �������� � �������� �����
				_set_errno(0);
				SetColor(12);  cout << "\n  ������ ��� �������� ����� ";  SetColor(15);  cout << HelpFileName;  SetColor(7);
				cout << "\n������� ENTER ��� �����������" << endl;
				PressENTER();
			}
			else {   //  ���� ���� ������� ������ � ������ ��� ������, ��
				heading += txt;
				if (fputs(heading.c_str(), HelpFilePointer) >= 0) {  //  ���������� ����� � ����       
					SetColor(10);  cout << "\n���� �������  "; SetColor(15);  cout << HelpFileName;  SetColor(10);  cout << "  ������� �������." << endl; SetColor(7);
					_fcloseall();
					cout << "\n������� ENTER ��� �����������" << endl;
					PressENTER();
				}
				else {
					SetColor(12);   cout << "\n  ������ ��� ������ � ���� ";   SetColor(15);   cout << HelpFileName;  SetColor(7);
					_fcloseall();
					cout << "\n������� ENTER ��� �����������" << endl;
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
// ��������� ����� ����������� ������ � ����. Black = 0, Blue = 1, Green = 2, Cyan = 3, Red = 4, Magenta = 5, Brown = 6, LightGray = 7,
//  DarkGray = 8, LightBlue = 9, LightGreen = 10, LightCyan = 11, LightRed = 12, LightMagenta = 13, Yellow = 14, White = 15. 
//  ���������� ���� ������ - SetColor(7,0).
void SetColor(int TextColor, int bg)
{
	HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hStdOut, (WORD)((bg << 4) | TextColor));
}
//
// ���� ����� ���� int � ���������
int getInt()
{
	while (true) // ���� ������������ �� ��� ���, ���� ������������ �� ����� ���������� ��������
	{
		int a;
		cin >> a;
		// �������� �� ���������� ����������
		if (cin.fail()) // ���� ���������� ���������� ��������� ���������,
		{
			cin.clear(); // �� ���������� cin � '�������' ����� ������
			cin.ignore(LLONG_MAX, '\n');    // � ������� �������� ����������� ����� �� �������� ������   
			cout << "������!!! ������ ���� ����� �����. ���������� ��� ���." << endl;
		}
		else
		{
			cin.ignore(LLONG_MAX, '\n'); // ������� ������ ��������
			return a;
		}
	}
}
//
// ���� ����� ���� int � ���������: ������������ �������� ������ �� Min �� Max ������������.
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
		cout << "������!!! ����� ������ ���� �� " << Min << " �� " << Max << ". ���������� ��� ���" << endl;
	}
	return tmp;
}
//
// ������� ������� ������� ENTER. ��� ��������� ������� ������������, ����� �� ����������
void PressENTER() {
	while (true) {
		if (_getch() == 13) break;
	}
}
// 
//  �������� ������� ������� ��������� � ����� ������� ENTER ������� �� ��������� 
void PrintErrorAndExit(const string& Text)
{
	SetColor(12);   cout << Text << endl << endl;   SetColor(7);
	cout << "   ������� ENTER  (����� �� ���������)" << endl;
	PressENTER();
	exit(0);
}
//
// ��������� �������� �� ������������ ����������� � �������� ��� Global Parameter 1 � Global Parameter 2 (�� IGES Specification Version 6.0, 2.2.3.1).
// ���������� true  ���� ������ �������������, �����- false.
bool CheckGP1GP2(const char& ch)
{
	if ((ch >= 0 && ch <= 32) || ch == 127 || (ch >= 48 && ch <= 57) || ch == 43 || ch == 45 || ch == 46 || ch == 68 || ch == 69 || ch == 72) return false;
	else return true;
}
//
// ������ � ��������� g_GParameters ���������� ��������� �� G_SectionVector, ��������� �� �� IGES Specification V.6.0. 
// ��������� ���������� � ���������� ���������� � InfoFileText. 
void ReadAndCheckGlobalParameters(const vector<string>& G_SectionVector, string& InfoFileText)
{
	// ������ Parameter Delimiter � Record Delimiter  � ������������ � �.2.2.3.1  "The Initial Graphics Exchange Specification(IGES) Version 6.0"
	int i = 0;  // ������ ������� � ������  G_SectionVector
	// ������ Global Parameter 1 (Parameter Delimiter) 
	if (G_SectionVector[0][0] == ',') 	Global_GlobalParameters.GP1_ParameterDelimiter = ',';
	else if (G_SectionVector[0][0] == '1' && G_SectionVector[0][1] == 'H') {
		Global_GlobalParameters.GP1_ParameterDelimiter = G_SectionVector[0][2];
		i += 3;
	}
	else PrintErrorAndExit("\n  ������!  �� ������ Global Parameter 1 (Parameter Delimiter).");
	// ��������� �� ������������ �������� Global Parameter 1 (Parameter Delimiter) 
	if (!CheckGP1GP2(Global_GlobalParameters.GP1_ParameterDelimiter))
		PrintErrorAndExit("������!  �������� �������� Global Parameter 1 (Parameter Delimiter)");
	// ��������� ����������� Parameter Delimiter ����� Global Parameter 1
	if (G_SectionVector[0][i] != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� ��������  Parameter Delimiter  �����  Global Parameter 1.");
	i++;
	//  ������ Global Parameter 2 (Record Delimiter)
	if (G_SectionVector[0][i] == Global_GlobalParameters.GP1_ParameterDelimiter)  Global_GlobalParameters.GP2_RecordDelimiter = ';';  // �������� �� ���������
	else if (G_SectionVector[0][i] == '1' && G_SectionVector[0][i + 1] == 'H') {
		Global_GlobalParameters.GP2_RecordDelimiter = G_SectionVector[0][i + 2];
		i += 3;
	}
	// ��������� �� ������������ �������� Global Parameter 2 (Record Delimiter)
	if (!CheckGP1GP2(Global_GlobalParameters.GP2_RecordDelimiter))
		PrintErrorAndExit("������!  �������� �������� Global Parameter 2 (Record Delimiter)");
	// ������ ����������� ����� Global Parameter 2 
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 2"); // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������  Global Parameter 3 (Product identification)
	Global_GlobalParameters.GP3_ProductID = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// ������ ����������� �����  Global Parameter 3
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 3"); // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������  Global Parameter 4 (File Name)
	Global_GlobalParameters.GP4_FileName = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// ������ ����������� �����  Global Parameter 4
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 4");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������  Global Parameter 5 (Native System ID)
	Global_GlobalParameters.GP5_NativeSystemID = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// ������ ����������� �����  Global Parameter 5
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 5");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������  Global Parameter 6 (Preprocessor version). 
	Global_GlobalParameters.GP6_PreprocessorVersion = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// ������ ����������� �����  Global Parameter 6
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 6");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 7 (Number of Binary Bits for Integer Representation)
	Global_GlobalParameters.GP7_BitsForInteger = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// ������ ����������� �����  Global Parameter 7
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 7");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 8 (Single-Precision Magnitud - Maximum power of ten representable in a single-precision floating point number)
	Global_GlobalParameters.GP8_SinglePrecisionMagnitud = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// ������ ����������� �����  Global Parameter 8
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 8");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 9 (Single-Precision Significance - Number of significant digits in a single-precision floating point number)
	Global_GlobalParameters.GP9_SinglePrecisionSignificance = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// ������ ����������� �����  Global Parameter 9
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 9");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 10 (Double-Precision Magnitude - Maximum power of ten representable in a double-precision floating point number)
	Global_GlobalParameters.GP10_DoublePrecisionMagnitude = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// ������ ����������� �����  Global Parameter 10
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 10");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 11 (Double-Precision Significance - Number of significant digits in a double-precision floating point number)
	Global_GlobalParameters.GP11_DoublePrecisionSignificance = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// ������ ����������� �����  Global Parameter 11
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 11");   // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 12 (Product Identification for the Receiver. The default value is the value specified in parameter 3)
	if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 12");  // ���� ��� Record Delimiter, �� ������ ������ � �����
	else if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP1_ParameterDelimiter)
		Global_GlobalParameters.GP12_ProductIdForReceiver = Global_GlobalParameters.GP3_ProductID;   // ���� ��� Parameter Delimiter, �� ����������� �������� �� ���������
	else 	// ����� - ������ ��������
		Global_GlobalParameters.GP12_ProductIdForReceiver = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// ������ ����������� �����  Global Parameter 12
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 12");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 13 (Model Space Scale. The default value is 1.0)
	if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 13");  // ���� ��� Record Delimiter, �� ������ ������ � �����
	else if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP1_ParameterDelimiter)
		Global_GlobalParameters.GP13_ModelSpaceScale = 1.0;   // ���� ��� Parameter Delimiter, �� ����������� �������� �� ���������
	else   // ����� - ������ ��������
		Global_GlobalParameters.GP13_ModelSpaceScale = ReadRealDataType(G_SectionVector, i, 'G');
	// ������ ����������� �����  Global Parameter 13
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 13");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 14 (Units Flag - ��� ������� ���������). default = 1 (inches);  2 - millimeters;  6 - meters.
	if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 14");  // ���� ��� Record Delimiter, �� ������ ������ � �����
	else if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP1_ParameterDelimiter)  // ���� ��� Parameter Delimiter,
		Global_GlobalParameters.GP14_UnitsFlag = 1;                                          //  �� ����������� �������� �� ���������
	else   // ����� - ������ ��������
		Global_GlobalParameters.GP14_UnitsFlag = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// ������ ����������� �����  Global Parameter 14
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 14");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 15 (Units Name)
	if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 15");  // ���� ��� Record Delimiter, �� ������ ������ � �����
	else if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP1_ParameterDelimiter)
		Global_GlobalParameters.GP15_UnitsName = "INCH";   // ���� ��� Parameter Delimiter, �� ����������� �������� �� ���������
	else   // ����� - ������ ��������
		Global_GlobalParameters.GP15_UnitsName = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// ������ ����������� �����  Global Parameter 15
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 15");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 16 (Maximum Number of Line Weight Gradations).
	if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 16");  // ���� ��� Record Delimiter, �� ������ ������ � �����
	else if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP1_ParameterDelimiter)
		Global_GlobalParameters.GP16_MaxNumLineWeightGradations = 1;   // ���� ��� Parameter Delimiter, �� ����������� �������� �� ���������
	else   // ����� - ������ ��������
		Global_GlobalParameters.GP16_MaxNumLineWeightGradations = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
	// ������ ����������� �����  Global Parameter 16
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 16");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 17 (Width of Maximum Line Weight in Units)
	Global_GlobalParameters.GP17_MaximumLineWidth = ReadRealDataType(G_SectionVector, i, 'G');
	// ������ ����������� �����  Global Parameter 17
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 17");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������  Global Parameter 18 (Date and Time of Exchange File Generation)
	Global_GlobalParameters.GP18_DateTimeFileGeneration = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
	// ������ ����������� �����  Global Parameter 18
	if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP1_ParameterDelimiter)
		PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 18");  // ���� ��� �� Parameter Delimiter, �� ������ ������ � ����� 
	// ������ Global Parameter 19 (Minimum User-Intended Resolution)
	Global_GlobalParameters.GP19_MinimumResolution = ReadRealDataType(G_SectionVector, i, 'G');
	// ��������� ��� ��������� ���������� ��������� ����� �������� �� ���������, �� ����������� �� �������� �� ���������
	Global_GlobalParameters.GP20_MaxCoordValue = 0.0;
	Global_GlobalParameters.GP21_AuthorName = "";
	Global_GlobalParameters.GP22_AuthorsOrganization = "";
	Global_GlobalParameters.GP23_VersionFlag = 3;
	Global_GlobalParameters.GP24_DraftingStandardFlag = 0;
	Global_GlobalParameters.GP25_DateTimeNativModelModified = "";
	Global_GlobalParameters.GP26_AppProtocolIdentifier = "";
	// ���� ��� ������ ���������� ���������� � 20-�� �� 26-�. ����������� 1 ���. ������ ������������ ����� Record Delimiter
	while (true) {
		char TmpDelimiter;  // ��������� ������ ��� ������� ������������ �����������
		// ������ ����������� �����  Global Parameter 19
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter
			PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 19");  // �� ������ ������ � �����. 
		// ������ Global Parameter 20 (Approximate Maximum Coordinate Value)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter, �� ������ ��������
			Global_GlobalParameters.GP20_MaxCoordValue = ReadRealDataType(G_SectionVector, i, 'G');
		// ������ ����������� �����  Global Parameter 20
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter
			PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 20");  // �� ������ ������ � �����. 
		// ������ Global Parameter 21 (Name of Author)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter, �� ������ ��������
			Global_GlobalParameters.GP21_AuthorName = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
		// ������ ����������� �����  Global Parameter 21
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter
			PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 21");  // �� ������ ������ � �����. 
		// ������ Global Parameter 22 (Author�s Organization)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter, �� ������ ��������
			Global_GlobalParameters.GP22_AuthorsOrganization = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
		// ������ ����������� �����  Global Parameter 22
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter
			PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 22");  // �� ������ ������ � �����. 
		// ������ Global Parameter 23 (Version Flag)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter, �� ������ ��������
			Global_GlobalParameters.GP23_VersionFlag = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
		// ������ ����������� �����  Global Parameter 23
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter
			PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 23");  // �� ������ ������ � �����. 
		// ������ Global Parameter 24 (Drafting Standard Flag)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter, �� ������ ��������
			Global_GlobalParameters.GP24_DraftingStandardFlag = static_cast<int>(ReadIntegerDataType(G_SectionVector, i, 'G'));
		// ������ ����������� �����  Global Parameter 24
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter
			PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 24");  // �� ������ ������ � �����. 
		// ������ Global Parameter 25 (Date and Time Model was Created or Modified)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter, �� ������ ��������
			Global_GlobalParameters.GP25_DateTimeNativModelModified = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
		// ������ ����������� �����  Global Parameter 25
		TmpDelimiter = ReadDelimiters(G_SectionVector, i, 'G');
		if (TmpDelimiter == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (TmpDelimiter != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter
			PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 25");  // �� ������ ������ � �����. 
		// ������ Global Parameter 26 (Application Protocol/Subset Identifier)
		if (G_SectionVector[i / 72][i % 72] == Global_GlobalParameters.GP2_RecordDelimiter) break;  // ����������� ������ ���� Record Delimiter
		else if (G_SectionVector[i / 72][i % 72] != Global_GlobalParameters.GP1_ParameterDelimiter) // ���� �� Parameter Delimiter, �� ������ ��������
			Global_GlobalParameters.GP26_AppProtocolIdentifier = Read_String(G_SectionVector, Global_GlobalParameters.GP1_ParameterDelimiter, Global_GlobalParameters.GP2_RecordDelimiter, i, 'G');
		// ������ ����������� �����  Global Parameter 26
		if (ReadDelimiters(G_SectionVector, i, 'G') != Global_GlobalParameters.GP2_RecordDelimiter) // ���� �� Record Delimiter
			PrintErrorAndExit("������!  �������� �������� ����������� �����  Global Parameter 26");  // �� ������ ������ � �����. 
		break;
	}
	// --- ��������� ����������� ���������� ��������� ---
	// ��������� Global Parameter 13 (Model Space Scale - ������� ������������ ������). ������ ���� 1.0, ������ �������� �� ������������ - ������ � ����� �� ��������� 
	if (Global_GlobalParameters.GP13_ModelSpaceScale != 1.) {
		char buffer[_CVTBUFSIZE];
		if (_gcvt_s(buffer, _CVTBUFSIZE, Global_GlobalParameters.GP13_ModelSpaceScale, Global_MaxSignifDigits)) { // ���� ������ ��� ��������������� double � string, �� ������� �� ���������
			PrintErrorAndExit("  ������ ��� �������� �������� Global Parameter 13 (Model Space Scale). \n\
				 ������ ��� ��������������� double � string.");
		}
		string txt{ "" };
		txt = txt + "������! Model Space Scale (������� ������������ ������) � IGES-����� �� ����� 1.0. \n" +
			"���������� �������� 13 (Model Space Scale) = " + buffer +
			".\n�������� ������ ���� ����� 1.0.";
		PrintErrorAndExit(txt);
	}
	// 
	// ��������� Global Parameter 14 (Units Flag - ��� ������� ���������). ������ ���� 2 (millimeters) ��� 6 (meters). ����� - ������ � ����� �� ���������
	if (Global_GlobalParameters.GP14_UnitsFlag != 2 && Global_GlobalParameters.GP14_UnitsFlag != 6) {
		string txt{ "" };
		txt = txt + "������! ������� ��������� � IGES-����� �� ����� � �� ����������. \n" +
			"���������� �������� 14 (Units Flag - ��� ������� ���������) = " + to_string(Global_GlobalParameters.GP14_UnitsFlag) +
			".\n�������� ������ ���� ���� \'2\' (����������), ���� \'6\' (�����). ";
		PrintErrorAndExit(txt);
	}
	//
	// ��������� ������������ ���������� � ������ ��� ���������� ������ � ����
	// ���������� � InfoFileText �������� Global Parameter 1 (Parameter Delimiter) 
	InfoFileText = InfoFileText + "���������� �������� 1 (Parameter Delimiter - ����������� ����������) = " + Global_GlobalParameters.GP1_ParameterDelimiter + '\n';
	// ���������� � InfoFileText �������� Global Parameter 2 (Record Delimiter)
	InfoFileText = InfoFileText + "���������� �������� 2 (Record Delimiter - ����������� �������) = " + Global_GlobalParameters.GP2_RecordDelimiter + '\n';
	// ���������� � InfoFileText �������� Global Parameter 3 (Product identification)
	InfoFileText = InfoFileText + "���������� �������� 3 (Product identification from sending system) = " + Global_GlobalParameters.GP3_ProductID + '\n';
	// ���������� � InfoFileText �������� Global Parameter 4 (File Name)
	InfoFileText = InfoFileText + "���������� �������� 4 (File Name) = " + Global_GlobalParameters.GP4_FileName + '\n';
	// ���������� � InfoFileText �������� Global Parameter 5 (Native System ID)
	InfoFileText = InfoFileText + "���������� �������� 5 (Native System ID) = " + Global_GlobalParameters.GP5_NativeSystemID + '\n';
	// ���������� � InfoFileText �������� Global Parameter 6 (Preprocessor version)
	InfoFileText = InfoFileText + "���������� �������� 6 (Preprocessor version) = " + Global_GlobalParameters.GP6_PreprocessorVersion + '\n';
	// ���������� � InfoFileText �������� Global Parameter 7 (Number of Binary Bits for Integer Representation)
	InfoFileText = InfoFileText + "���������� �������� 7 (Number of Binary Bits for Integer Representation) = " + to_string(Global_GlobalParameters.GP7_BitsForInteger) + '\n';
	// ���������� � InfoFileText �������� Global Parameter 8 (Single-Precision Magnitud )
	InfoFileText = InfoFileText + "���������� �������� 8 (Single-Precision Magnitud - Maximum power of ten representable in a single-precision floating point number) = " + to_string(Global_GlobalParameters.GP8_SinglePrecisionMagnitud) + '\n';
	// ���������� � InfoFileText �������� Global Parameter 9 (Single-Precision Significance)
	InfoFileText = InfoFileText + "���������� �������� 9 (Single-Precision Significance - Number of significant digits in a single-precision floating point number) = " + to_string(Global_GlobalParameters.GP9_SinglePrecisionSignificance) + '\n';
	// ���������� � InfoFileText �������� Global Parameter 10 (Double-Precision Magnitude)
	InfoFileText = InfoFileText + "���������� �������� 10 (Double-Precision Magnitude - Maximum power of ten representable in a double-precision floating point number) = " + to_string(Global_GlobalParameters.GP10_DoublePrecisionMagnitude) + '\n';
	// ���������� � InfoFileText �������� Global Parameter 11 Double-Precision Significance)
	InfoFileText = InfoFileText + "���������� �������� 11 (Double-Precision Significance - Number of significant digits in a double-precision floating point number) = " + to_string(Global_GlobalParameters.GP11_DoublePrecisionSignificance) + '\n';
	// ���������� � InfoFileText �������� Global Parameter 12 (Product Identification for the Receiver)
	InfoFileText = InfoFileText + "���������� �������� 12 (Product Identification for the Receiver) = " + Global_GlobalParameters.GP12_ProductIdForReceiver + '\n';
	// ���������� � InfoFileText �������� Global Parameter 13 (Model Space Scale)
	{
		char buffer[_CVTBUFSIZE];
		// ���� ������ ��� ��������������� double � string, �� ������� �� ���������
		if (_gcvt_s(buffer, _CVTBUFSIZE, Global_GlobalParameters.GP13_ModelSpaceScale, Global_MaxSignifDigits)) {
			PrintErrorAndExit("  ������ ��� ������ ���������� � Global Parameter 13 (Model Space Scale). \n\
				 ������ ��� ��������������� double � string.");
		}
		InfoFileText = InfoFileText + "���������� �������� 13 (Model Space Scale) = " + buffer + '\n';
	}
	// ���������� � InfoFileText �������� Global Parameter 14 (Units Flag - ��� ������� ���������)
	InfoFileText = InfoFileText + "���������� �������� 14 (Units Flag - ��� ������� ���������) = " + to_string(Global_GlobalParameters.GP14_UnitsFlag) + '\n';
	// ���������� � InfoFileText �������� Global Parameter 15 (Units Name)
	InfoFileText = InfoFileText + "���������� �������� 15 (Units Name - ��� ������� ���������) = " + Global_GlobalParameters.GP15_UnitsName + '\n';
	// ���������� � InfoFileText �������� Global Parameter 16 (Maximum Number of Line Weight Gradations)
	InfoFileText = InfoFileText + "���������� �������� 16 (Maximum Number of Line Weight Gradations) = " + to_string(Global_GlobalParameters.GP16_MaxNumLineWeightGradations) + '\n';
	// ���������� � InfoFileText �������� Global Parameter 17 (Width of Maximum Line Weight in Units)
	{
		char buffer[_CVTBUFSIZE];
		// ���� ������ ��� ��������������� double � string, �� ������� �� ���������
		if (_gcvt_s(buffer, _CVTBUFSIZE, Global_GlobalParameters.GP17_MaximumLineWidth, Global_MaxSignifDigits)) {
			PrintErrorAndExit("������ ��� ������ ���������� � Global Parameter 17 (Maximum Line Width). \n\
				 ������ ��� ��������������� double � string.");
		}
		InfoFileText = InfoFileText + "���������� �������� 17 (Width of Maximum Line Weight in Units) = " + buffer + '\n';
	}
	// ���������� � InfoFileText �������� Global Parameter 18 (Date and Time of Exchange File Generation)
	InfoFileText = InfoFileText + "���������� �������� 18 (Date and Time of Exchange File Generation (YYYYMMDD:hhmmss or YYMMDD:hhmmss)) = " + Global_GlobalParameters.GP18_DateTimeFileGeneration + '\n';
	// ���������� � InfoFileText �������� Global Parameter 19 (Minimum User-Intended Resolution)
	{
		char buffer[_CVTBUFSIZE];
		// ���� ������ ��� ��������������� double � string, �� ������� �� ���������
		if (_gcvt_s(buffer, _CVTBUFSIZE, Global_GlobalParameters.GP19_MinimumResolution, Global_MaxSignifDigits)) {
			PrintErrorAndExit("������ ��� ������ ���������� � Global Parameter 19 (Minimum Resolution). \n\
				 ������ ��� ��������������� double � string.");
		}
		InfoFileText = InfoFileText + "���������� �������� 19 (Minimum User-Intended Resolution) = " + buffer + '\n';
	}
	// ���������� � InfoFileText �������� Global Parameter 20 (Approximate Maximum Coordinate Value)
	{
		char buffer[_CVTBUFSIZE];
		// ���� ������ ��� ��������������� double � string, �� ������� �� ���������
		if (_gcvt_s(buffer, _CVTBUFSIZE, Global_GlobalParameters.GP20_MaxCoordValue, Global_MaxSignifDigits)) {
			PrintErrorAndExit("������ ��� ������ ���������� � Global Parameter 20 (Approximate Maximum \n\
		Coordinate Value). ������ ��� ��������������� double � string.");
		}
		InfoFileText = InfoFileText + "���������� �������� 20 (Approximate Maximum Coordinate Value) = " + buffer + '\n';
	}
	// ���������� � InfoFileText �������� Global Parameter 21 (Name of Author)
	InfoFileText = InfoFileText + "���������� �������� 21 (Name of Author) = " + Global_GlobalParameters.GP21_AuthorName + '\n';
	// ���������� � InfoFileText �������� Global Parameter 22 (Author�s Organization)
	InfoFileText = InfoFileText + "���������� �������� 22 (Author�s Organization) = " + Global_GlobalParameters.GP22_AuthorsOrganization + '\n';
	// ���������� � InfoFileText �������� Global Parameter 23 (Version Flag)
	InfoFileText = InfoFileText + "���������� �������� 23 (Version Flag) = " + to_string(Global_GlobalParameters.GP23_VersionFlag) + '\n';
	// ���������� � InfoFileText �������� Global Parameter 24 (Drafting Standard Flag)
	InfoFileText = InfoFileText + "���������� �������� 24 (Drafting Standard Flag) = " + to_string(Global_GlobalParameters.GP24_DraftingStandardFlag) + '\n';
	// ���������� � InfoFileText �������� Global Parameter 25 (Date and Time Model was Created or Modified)
	InfoFileText = InfoFileText + "���������� �������� 25 (Date and Time Model was Created or Modified) = " + Global_GlobalParameters.GP25_DateTimeNativModelModified + '\n';
	// ���������� � InfoFileText �������� Global Parameter 26 (Application Protocol/Subset Identifier)
	InfoFileText = InfoFileText + "���������� �������� 26 (Application Protocol/Subset Identifier) = " + Global_GlobalParameters.GP26_AppProtocolIdentifier + '\n';
}  // ����� ������� ReadAndCheckGlobalParameters()
//
// ������ ������ ����������� (Parameter Delimiter  ���  Record Delimiter) ������� � ������� � �������� i � �������� Global 
// Section  �  Parameter Data Section. ������������� �������� i �� ������, ��������� �� ������������ ��� �� ������������
// ��������� (���� ��� ����). ��������  Section  ���������� ������ ('G'- Global Section, 'P'- Parameter Data Section).
// ���������� ����������� ������.
char ReadDelimiters(const vector<string>& SectionVector, int& i, const char& Section)
{
	int StrLength;  // ����� ������ ����������, ������ ���������� ������. ����� 72 ��� ������� 'G'(Global Section). ����� 64 ��� ������� 'P' (Parameter Data Section)
	// ��������� �������� Section (������ ���� ���� 'G', ���� 'P')
	if (Section != 'G' && Section != 'P')
		PrintErrorAndExit("������ � ����! �������� �4 � ������� ReadDelimiters() ���������� �� \'G\' � \'P\'");
	Section == 'G' ? StrLength = Global_GSectionLineLength : StrLength = Global_PSectionLineLength;
	//  ���������, �������� �� ������ ������������ 
	if (SectionVector[i / StrLength][i % StrLength] != Global_GlobalParameters.GP1_ParameterDelimiter &&
		SectionVector[i / StrLength][i % StrLength] != Global_GlobalParameters.GP2_RecordDelimiter)
	{
		SetColor(12);   cout << "\n������ ��� ������ ����������� � �������  ";
		(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
		cout << "������ �" << (i % StrLength) + 1 << " � ������ � " << (i / StrLength) + 1 << " ������� ";
		(Section == 'G') ? cout << "Global Section  " : cout << "Parameter Data Section  ";
		cout << "�� �������� ������������." << endl << endl;   SetColor(7);
		cout << "   ������� ENTER  (����� �� ���������)" << endl;
		PressENTER();
		exit(0);
	}
	// ���� ������ �������� ������������
	// ���� ������ �������� Record Delimiter, ���������� ������  Record Delimiter (����� Record Delimiter ������ �� ������)
	if (SectionVector[i / StrLength][i % StrLength] == Global_GlobalParameters.GP2_RecordDelimiter) return Global_GlobalParameters.GP2_RecordDelimiter;
	// ����� (�.�. ���� ������ �������� Parameter Delimiter)
	i++;  //  ��������� �� ��������� ������
	//  ���������� ��� ������� '������' (���� ��� ����)
	while (SectionVector[i / StrLength][i % StrLength] == ' ') i++;
	return Global_GlobalParameters.GP1_ParameterDelimiter;  // ���������� ������   Parameter Delimiter
}  // ����� ������� ReadDelimiters()
//
// ������ � ���������� �������� ���� String, ������� ������ � ������� i � �������� Global Section  �  Parameter Data
// Section. ������ �������� ������� i �� ������, ��������� �� ����������� �������. ��������  Section  ���������� ������,
// � ������� ���������� ������ ('G'- Global Section, 'P'- Parameter Data Section).
string Read_String(const vector<string>& SectionVector, const char& GP1_ParameterDelimiter, const char& GP2_RecordDelimiter, int& i, const char& Section)
{
	string GP_String;  // ������������ �������� - ���������� �������� ���� String
	string LengthString; // ����� ������. C��� �������� ����� - ������ ����� String-��������� ����� �������� 'H', �.� ���������� �������� ��� ������.
	int StrLength;  // ����� ������ ����������, ������ ���������� ������. ����� 72 ��� ������� 'G'(Global Section). ����� 64 ��� ������� 'P' (Parameter Data Section)
	// ��������� �������� Section
	if (Section != 'G' && Section != 'P')
		PrintErrorAndExit("������ � ����! �������� �4 � ������� Read_String_From_GSection() �� \'G\' � �� \'P\'");
	Section == 'G' ? StrLength = Global_GSectionLineLength : StrLength = Global_PSectionLineLength;
	// ������ � ������ LengthString ����� � ������ String-��������� ����� �������� 'H' (�.�. ������ ����� ������)
	while (true) {
		if (SectionVector[i / StrLength][i % StrLength] >= '0' && SectionVector[i / StrLength][i % StrLength] <= '9' &&    // ���� ������� � ��������� ������ - �����, �� ... 
			SectionVector[i / StrLength][i % StrLength + 1] >= '0' && SectionVector[i / StrLength][i % StrLength + 1] <= '9')
		{
			LengthString += SectionVector[i / StrLength][i % StrLength];   //  ... �� ���������� ������� ������ (�����) � ������ LengthString, � ...
			i++;  // ... � ��������� �� ��������� ������
		}
		else if (SectionVector[i / StrLength][i % StrLength] >= '0' && SectionVector[i / StrLength][i % StrLength] <= '9' &&    // ���� ������� ������ - �����, � ...
			SectionVector[i / StrLength][i % StrLength + 1] == 'H')    // ... � ��������� ������ - 'H',  �� ...
		{
			LengthString += SectionVector[i / StrLength][i % StrLength];   //  ... �� ���������� ������� ������ (�����), � ...
			i += 2;  // ... ��������� �� ������, ��������� �� �������� 'H',  � ...
			break;  // ... ���������� ������ ����� � ������ LengthString.
		}
		else {  //  ����� - ��������� �� ������ � ����� �� ���������
			SetColor(12);   cout << "\n������ ��� ������ ����� � ������ String-���������." << endl;
			cout << "������ �" << (i % StrLength) + 1 << " � ������ � " << (i / StrLength) + 1 << " ������� ";
			(Section == 'G') ? cout << "Global Section  " : cout << "Parameter Data Section  ";
			cout << "�� �������� ������." << endl << endl;   SetColor(7);
			cout << "   ������� ENTER  (����� �� ���������)" << endl;
			PressENTER();
			exit(0);
		}
	}
	// ��������� ���������� ����� (����� ������) - �� ������ ���� ����� 0. 
	if (stoi(LengthString) == 0) {  // ���� ������� ����� ������ = 0
		SetColor(12);   cout << "\n������! ����� � ������ String-��������� (����� �������� \'H'\) ����� = 0." << endl;
		cout << "�������� ������ �" << ((i - 2) % StrLength) + 1 << " � ������ � " << ((i - 2) / StrLength) + 1 << endl << endl;   SetColor(7);
		cout << "   ������� ENTER  (����� �� ���������)" << endl;
		PressENTER();
		exit(0);
	}
	// ������ ����� ��������� (�.�. ����� ����� ������� 'H')
	for (int j = 0; j < stoi(LengthString); j++) {
		// ���� ������ �� ����������� ���������, �� ������ ���.
		if (!(SectionVector[i / StrLength][i % StrLength] >= 0 && SectionVector[i / StrLength][i % StrLength] < 32) &&
			SectionVector[i / StrLength][i % StrLength] != 127)
		{
			GP_String += SectionVector[i / StrLength][i % StrLength];
			i++;
		}
		else {  // ����� (�.�. ���� ������ �� �� ����������� ���������) - ��������� �� ������ � ����� �� ���������
			SetColor(12);   cout << "\n������! �������� ������ String-��������� � �������  ";
			(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
			cout << "�������� ������ �" << (i % StrLength) + 1 << " � ������ � " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
			cout << "   ������� ENTER  (����� �� ���������)" << endl;
			PressENTER();
			exit(0);
		}
	}
	return GP_String;
}   // Read_String()
//
// ������ � ���������� ����� ����� ������� ������ � ������� i � �������� Global Section  �  Parameter Data Section.
//  ������ �������� ������� i �� ������, ��������� �� ����������� �������. ��������  Section  ���������� ������,
//  � ������� ���������� ������ ('G'- Global Section, 'P'- Parameter Data Section).
long long  ReadIntegerDataType(const vector<string>& SectionVector, int& i, const char& Section)
{
	string Txt;  // �����, ���� ������� �������� ����� ����� ������������ � long long
	int StrLength;  // ����� ������ ����������, ������ ���������� ������. ����� 72 ��� ������� 'G'(Global Section). ����� 64 ��� ������� 'P' (Parameter Data Section)
	// ��������� �������� Section
	if (Section != 'G' && Section != 'P')
		PrintErrorAndExit("������ � ����! �������� �3 � ������� ReadInteger() �� \'G\' � �� \'P\'");
	Section == 'G' ? StrLength = Global_GSectionLineLength : StrLength = Global_PSectionLineLength;
	// ������ ������ ������ - ���� ����� (���� �� ����)
	if (SectionVector[i / StrLength][i % StrLength] == '+' || SectionVector[i / StrLength][i % StrLength] == '-') {
		Txt = SectionVector[i / StrLength][i % StrLength];
		i++;
	}
	// ���������, ��� ������ ������ - �����
	if (SectionVector[i / StrLength][i % StrLength] < '0' || SectionVector[i / StrLength][i % StrLength] > '9') {
		SetColor(12);   cout << "\n������! �������� ������ ��� ������ ����� � �������  ";
		(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
		cout << "�������� ������ �" << (i % StrLength) + 1 << " � ������ � " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
		cout << "   ������� ENTER  (����� �� ���������)" << endl;
		PressENTER();
		exit(0);
	}
	// ������ �����
	while (SectionVector[i / StrLength][i % StrLength] >= '0' && SectionVector[i / StrLength][i % StrLength] <= '9') {
		Txt = Txt + SectionVector[i / StrLength][i % StrLength];
		i++;
	}
	return stoll(Txt);
}  // ����� �������  ReadIntegerDataType()
//
// ������ � ���������� ����� � ��������� ������ ������� ������ � ������� i � �������� Global Section  �  Parameter Data 
// Section. ������ �������� ������� i �� ������, ��������� �� ����������� �������. ��������  Section  ���������� ������, 
// � ������� ���������� ������ ('G'- Global Section, 'P'- Parameter Data Section).
double ReadRealDataType(const vector<string>& SectionVector, int& i, const char& Section)
{
	double Result;  // ������������ ��������
	int SignificantDigitsNumber{ 0 };  // ���������� ���� � �������� ��� ����� ������� � ������ �����. ������ ���������� � ������ �������� ����� 
	bool DecimalPointExists{ false };  // ��������� ������������� ���������� �����. false - ����� �� ����������, true - ����������.  
	string RealNumber = "";
	unsigned int StrLength{ 0 };  // ����� ������ ����������, ������ ���������� ������. ����� 72 ��� ������� 'G'(Global Section). ����� 64 ��� ������� 'P' (Parameter Data Section)
	// ��������� �������� Section
	if (Section != 'G' && Section != 'P')
		PrintErrorAndExit("������ � ����! �������� �3 � ������� ReadRealDataType �� \'G\' � �� \'P\'");
	else Section == 'G' ? StrLength = Global_GSectionLineLength : StrLength = Global_PSectionLineLength;
	// ������ ������ ������ �������� - ���� ����� (���� �� ����)
	if (SectionVector[i / StrLength][i % StrLength] == '+') i++;  // ���� ������ ������ '+', �� ��� ����������
	else if (SectionVector[i / StrLength][i % StrLength] == '-') { // ���� ������ ������ '-', �� ���������� ���
		RealNumber = '-';
		i++;
	}
	// ��������� ���� ����� '+'���'-' (���� �� ���). ����� - ����� ����� ��������. ��� ����� ���� ����� �� 0 �� 9 ��� '.'. ���������, ����� ������ � ����� �� ���������
	else if ((SectionVector[i / StrLength][i % StrLength] < '0' || SectionVector[i / StrLength][i % StrLength] > '9') &&
		SectionVector[i / StrLength][i % StrLength] != '.') {
		SetColor(12);   cout << "\n������! �������� ������ ������ ����� ���� Real  \n� �������  ";
		(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
		cout << "�������� ������ �" << (i % StrLength) + 1 << " � ������ � " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
		cout << "   ������� ENTER  (����� �� ���������)" << endl;
		PressENTER();
		exit(0);
	}
	// ������ �������� ����� - ��� ����� ���� ����� �� 0 �� 9 ��� ���������� ����� '.' (���������� ����� ����� ���� �� ����� 1 ����)
	while (true) {
		if (SignificantDigitsNumber == 0 && SectionVector[i / StrLength][i % StrLength] == '0' &&     // ���������� ������� '0' � ������ ����� �� ���������� �����
			DecimalPointExists == false) {
			i++;
			continue;
		}
		else if (SectionVector[i / StrLength][i % StrLength] >= '0' && SectionVector[i / StrLength][i % StrLength] <= '9') {  // ������ ����� - �� 0 �� 9 ������������
			RealNumber = RealNumber + SectionVector[i / StrLength][i % StrLength];
			if (SectionVector[i / StrLength][i % StrLength] == '0' && SignificantDigitsNumber == 0) {}
			else SignificantDigitsNumber++;
			i++;
			continue;
		}
		else if (SectionVector[i / StrLength][i % StrLength] == '.') {  // ������ ���������� ����� '.'
			if (DecimalPointExists == true) {    // ���� ����������� ������ ���������� ����� - �� ������ � �����
				SetColor(12);   cout << "\n������ ��� ������ ����� ���� Real  � �������  ";
				(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
				cout << "� �������� ����� ���� ����� ����� ���������� �����." << endl;
				cout << "�������� ������ �" << (i % StrLength) + 1 << " � ������ � " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
				cout << "   ������� ENTER  (����� �� ���������)" << endl;
				PressENTER();
				exit(0);
			}
			else if (SignificantDigitsNumber == 0) RealNumber = RealNumber + '0';   // ���� ����� ����� �������� ����������� ��� ����� 0, �� ��������� '0' ����� ���������� ������ 
			DecimalPointExists = true;
			RealNumber = RealNumber + ',';
			i++;
			continue;
		}
		// ��������� ��� �������� ����� ��������: ���� ������ �� ����� '.' � �� �����, �� ������ ���� 'E', 'e', 'D', 'd' ���  �����������
		if (SectionVector[i / StrLength][i % StrLength] == 'E' || SectionVector[i / StrLength][i % StrLength] == 'e' ||
			SectionVector[i / StrLength][i % StrLength] == 'D' || SectionVector[i / StrLength][i % StrLength] == 'd' ||
			SectionVector[i / StrLength][i % StrLength] == Global_GlobalParameters.GP1_ParameterDelimiter ||
			SectionVector[i / StrLength][i % StrLength] == Global_GlobalParameters.GP2_RecordDelimiter) {
		}
		else {
			SetColor(12);   cout << "\n������ ��� ������ ����� ���� Real � �������  ";
			(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
			cout << "������ ��� ������ �������� �����. �������� ������ �" << (i % StrLength) + 1 << " � ������ � " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
			cout << "   ������� ENTER  (����� �� ���������)" << endl;
			PressENTER();
			exit(0);
		}
		break;  // ����� �� ����� ������ �������� � ����������� ������
	}   //  ��������� ������ �������� �����
	// ��� ����� � ���������� ������ ������� ���� � ����� �������� ����� ���������� �����   
	if (DecimalPointExists == true) {
		while (RealNumber[RealNumber.size() - 1] == '0') {
			RealNumber.resize(RealNumber.size() - 1);
			SignificantDigitsNumber--;
		}
	}
	// ������ ���������� - ������ ����� ����� ����� ������� 'E', 'e', 'D' ��� 'd', ���� ��� ����
	if (SectionVector[i / StrLength][i % StrLength] == 'E' || SectionVector[i / StrLength][i % StrLength] == 'e' ||
		SectionVector[i / StrLength][i % StrLength] == 'D' || SectionVector[i / StrLength][i % StrLength] == 'd') {
		RealNumber = RealNumber + 'E';
		i++;
		// ������ ���� (���� �� ����) ����� ����������� 
		if (SectionVector[i / StrLength][i % StrLength] == '+' || SectionVector[i / StrLength][i % StrLength] == '-') {
			RealNumber = RealNumber + SectionVector[i / StrLength][i % StrLength];
			i++;
		}
		// ��������� ��� ������ ������ ���������� - �����
		if (SectionVector[i / StrLength][i % StrLength] < '0' || SectionVector[i / StrLength][i % StrLength] > '9') {
			SetColor(12);   cout << "\n������! �������� ������ ��� ������ ���� ���������� ����� \n� �������  ";
			(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
			cout << "�������� ������ �" << (i % StrLength) + 1 << " � ������ � " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
			cout << "   ������� ENTER  (����� �� ���������)" << endl;
			PressENTER();
			exit(0);
		}
		// ������ ����� ����������
		while (SectionVector[i / StrLength][i % StrLength] >= '0' && SectionVector[i / StrLength][i % StrLength] <= '9') {
			RealNumber = RealNumber + SectionVector[i / StrLength][i % StrLength];
			i++;
		}
		// ��������� ��� �������� ����� ����������: ���� ������ �� ����������� - �� ������ � �����     
		if (SectionVector[i / StrLength][i % StrLength] == Global_GlobalParameters.GP1_ParameterDelimiter ||
			SectionVector[i / StrLength][i % StrLength] == Global_GlobalParameters.GP2_RecordDelimiter) {
		}
		else {
			SetColor(12);   cout << "\n������ ��� ������ ����� ���� Real � ������� ";
			(Section == 'G') ? cout << "Global Section.\n" : cout << "Parameter Data Section.\n";
			cout << "������ ��� ������ ���������� �����. �������� ������ �" << (i % StrLength) + 1 << " � ������ � " << (i / StrLength) + 1 << endl << endl;   SetColor(7);
			cout << "   ������� ENTER  (����� �� ���������)" << endl;
			PressENTER();
			exit(0);
		}

	}
	// ��������� ���������
	Result = atof(RealNumber.c_str());
	// ���� ���������� ������ �������� - �������� ��������� �� ����
	if (SignificantDigitsNumber > Global_MaxSignifDigits) {
		char buffer[_CVTBUFSIZE];
		if (_gcvt_s(buffer, _CVTBUFSIZE, Result, Global_MaxSignifDigits)) {  // ���� ������ ��� ��������������� double � string, �� ������� �� ���������
			PrintErrorAndExit("������ � ReadRealDataType() ��� ������ _gcvt_s() ��� ������ ��������.");
		}
		SetColor(12);   cout << "\n��������!";  SetColor(14);   cout << "  ������ �������� ��� ������ ����� � ��������� �������." << endl;
		cout << "��������� ���������� ������� �����:  ������  ";  (Section == 'G') ? cout << "Global Section,\n" : cout << "Parameter Data Section,\n";
		cout << "������ � " << (i / StrLength) + 1 << ", ������ �" << (i % StrLength) << endl;
		cout << "� ���������� �������� ������ �����  " << RealNumber << endl;
		cout << "����� �������������� �����  " << buffer << "." << endl << endl;  SetColor(7);
	}
	return Result;
}   //  ����� �������  ReadRealDataType()
//
// 
// ������ � ���������� ����� ����� � ������� Directory Entry ��� ������� �� ������ D_Sect_Str_Number � ���� FieldNumber.
// ���� ���� ��������� ��������� - ���������� 0. ���� � ���� ��������� ����� ������ ������ ����� - ������ � ����� �� ���������
int ReadIntegerIn_D_Section(const vector<string>& DSectionVector, const int D_Sect_Str_Number, const int FieldNumber)
{
	// static int N{ 0 };    // �������
	// N++;   // �������
	// cout << "ReadIntegerIn_D_Section().  N= " << N << ". D_Sect_Str_Number= " << D_Sect_Str_Number << ". FieldNumber= " << FieldNumber;   // �������
	if (D_Sect_Str_Number % 2 == 0) {    // ���� ��������� ����� ������ D_Sect_Str_Number - ������ �����, �� ������ � ����� �� ���������
		string txt = "\n������ ��� ������ ������� ReadIntegerIn_D_Section().\n�������� ������ �������� D_Sect_Str_Number. �������� �������� " + to_string(D_Sect_Str_Number) +
			" \n(�������� ������ ���� �������� ������).";
		// cout << "\nFieldNumber = " << FieldNumber << endl;   //  �������
		PrintErrorAndExit(txt);
	}
	int Result;  // ������������ ��������
	int DSectionStringIndex = D_Sect_Str_Number - 1 + static_cast<int>(FieldNumber / 10); // ������ ������ � ������� DSectionVector, ��� ����� ��������� �����
	// cout << "������� ReadIntegerIn_D_Section().  DSectionStringIndex = " << DSectionStringIndex << endl;   // �������
	int StartIndex = (Global_D_Section_Field_Size * (FieldNumber - 1)) % Global_IGES_File_StringLength;  // ������ ������� � ������ ������, � �������� ����� ������ ������ �����
	// cout << "������� ReadIntegerIn_D_Section().  StartIndex = " << StartIndex << endl;   // �������
	string TxtInField = DSectionVector[DSectionStringIndex].substr(StartIndex, Global_D_Section_Field_Size);  // ������ � ������ TxtInField ����� �� ���������� ����
	// cout << "������� ReadIntegerIn_D_Section().  TxtInField = " << TxtInField << endl;   // �������   
	if (TxtInField == "        ") {
		// cout << " Result= 8_��������. " << endl;   // �������
		return 0;   // ���� ���� ������� �� 8-�� �������� - ���������� 0 
	}
	// ���������, ��� ��� 8 �������� � TxtInField - ��� ����� ����� (�� ������ ��� ���)
	{
		int i = 0;
		for (; i < 8; i++) {   // ���������� ������� � ������
			if (TxtInField[i] != ' ') break;
		}
		if ((i < 7 && TxtInField[i] == '+') || (i < 7 && TxtInField[i] == '-')) i++;  // ���������� ���� + ��� - (���� �� ����)
		for (; i < 8; i++) {   // ���������, ��� ������� - ��� ����� �� 0 �� 9
			if (TxtInField[i] < '0' || TxtInField[i] > '9') break;
		}
		if (i < 8) {  // ���� ��������� ����������� ������
			string txt = "\n������ � �������� �����: \n� Directory Entry Section �������� �������� � �������� �� ������ " +
				to_string(D_Sect_Str_Number) + ", � ���� " + to_string(FieldNumber) + ".\n���� �������� ����� \"" + TxtInField + "\" (������ ���� ����� ����� � ������� ���� ����).";
			PrintErrorAndExit(txt);
		}
	}
	try {
		Result = stoi(DSectionVector[DSectionStringIndex].substr(StartIndex, Global_D_Section_Field_Size));
	}
	catch (invalid_argument const& ex) {
		string txt = "\n������ � �������� �����: \n� Directory Entry Section �������� �������� � �������� �� ������ " +
			to_string(D_Sect_Str_Number) + ", � ���� " + to_string(FieldNumber) + ".\n���� �������� ����� \"" + TxtInField + "\" (������ ���� ����� �����).";
		PrintErrorAndExit(txt);
	}
	// cout << " Result= " << Result << endl;   // �������
	return Result;
}
//
// 
// ������ ���� (�.�. �������������� ���� 13 - Color Number) ��� ������ Entity �� ������ Entity_D_Sect_Str_Number. 
// ����������: "����������" ���� (0-8), ��� ������������� ���� - 10-������� ����� ������� 1RGB (�������� ���  
// RGB:254,001,220 ��� 1254001220). "����������": 0-no color(default), 1-Black(RGB:0,0,0), 2-Red (RGB:255,0,0), 3-Green  
// (RGB:0,255,0), 4-Blue(RGB:0,0,255), 5-Yellow(RGB:255,255,0), 6-Magenta (RGB:255,0,255), 7-Cyan (RGB:0,255,255), 8-White (RGB:255,255,255).
int ReadEntityColor(const vector<string>& D_SectionVector, const vector<string>& P_SectionVector, const int D_Sect_Str_Number)
{
	int EntityField13_ColorNumber = ReadIntegerIn_D_Section(D_SectionVector, D_Sect_Str_Number, 13);  // ����� � ���� 13 (Color Number).
		// stoi(D_SectionVector[D_Sect_Str_Number].substr(16, 8)); // ����� � ���� 13 (Color Number).    �������
	int ColorField2_ParameterDataLine;  // ����� � ���� 2 ��� ������� Type 314 "Color Entity" - ��������� �� ������ � Parameter Data ��� Color
	int ColorTypeNumber_D_Section{ 0 };  // ����� ������� �� ������ (������ ���� 314 "Color Definition Entity") � ������� D_Section
	int ColorTypeNumber_P_Section{ 0 };  // ����� ������� �� ������ (������ ���� 314 "Color Definition Entity") � ������� P_Section
	double ColorCoordinate1_RED{ 0 }, ColorCoordinate2_GREEN{ 0 }, ColorCoordinate3_BLUE{ 0 };  // ����������� �� P_Section ��� Color �������� RGB
	int Color_R{ 0 }, Color_G{ 0 }, Color_B{ 0 };  // �������� RGB ��� ����� (�� 0 �� 255)
	// ���� � ���� 13 ����� ������ 8 ��� ������������� � ������� ������ ������� Directory Entry, �� ��������� �� ������ � ����� �� ���������
	if (EntityField13_ColorNumber > 8 || (EntityField13_ColorNumber < (-1) * static_cast<int>(D_SectionVector.size()))) {
		string txt = "\n������ � �������� �����:\n  Directory Entry Section, ������ �� ������ " + to_string(D_Sect_Str_Number) +
			", �������� � ���� 13 \"Color Number\" = " + to_string(EntityField13_ColorNumber) +
			".\n������ ���� ������/����� 8 ��� ������������� �� ������ ������ ������� Directory Entry.";
		PrintErrorAndExit(txt);
	}
	else if (EntityField13_ColorNumber >= 0 && EntityField13_ColorNumber <= 8) return EntityField13_ColorNumber;  // ���������� ����� ����� ���� � Entity � ���� 13 ������ "�����������" ����
	// ����� (�.�. ���� � ���� 13 ������������� �����) - ������ ���� � ������� Type 314 �� ������ �� ������ "-Field13"
	ColorTypeNumber_D_Section = ReadIntegerIn_D_Section(D_SectionVector, -EntityField13_ColorNumber, 1);
	// stoi(D_SectionVector[-EntityField13_ColorNumber - 1].substr(0, 8));    // �������
// ���� ���� 1 �� ��������� ������ �� ������ 314 - �� ��������� �� ������ � ����� �� ���������
	if (ColorTypeNumber_D_Section != Global_Entity314ColorNumber) {
		string txt = "\n������ � �������� �����:\n  Directory Entry Section, ������ �� ������ " + to_string(-EntityField13_ColorNumber) +
			" ����� ����� " + to_string(ColorTypeNumber_D_Section) + " (������ ���� 314).";
		PrintErrorAndExit(txt);
	}
	// ����� (���� ������ ��������� �� ������ Entity 314) - ������ RGB ��� ����� � Parameter Data Section
	// � ���� 2 ������� 314 "Color" ������ Pointer, ����������� �� ������ � Parameter Data Section
	ColorField2_ParameterDataLine = ReadIntegerIn_D_Section(D_SectionVector, -EntityField13_ColorNumber, 2);
	// stoi(D_SectionVector[-EntityField13_ColorNumber - 1].substr(8, 8));     // �������
// �������� �������� � ���� 2 ��� ������� Entity 314.  �������� ������ ���� ������ 0 � ������/����� ������� Parameter Data Section
	if (ColorField2_ParameterDataLine < 1 || ColorField2_ParameterDataLine > P_SectionVector.size()) {
		string txt = "\n������ � �������� �����:\n������ Directory Entry Section, ������ � ���� 2 �� ������ " + to_string(-EntityField13_ColorNumber) +
			".\n������� �������� " + to_string(ColorField2_ParameterDataLine) + " (������ ���� ����� 0 � �����/����� ������� Parameter Data Section).";
		PrintErrorAndExit(txt);
	}
	//  --- ������ RGB ��� ����� � Parameter Data Section --- 
	int i = (ColorField2_ParameterDataLine - 1) * Global_PSectionLineLength;  // ��������� "�������" � Parameter Data Section
	// ������ ������ �������� � Parameter Data Section - Entity Type Number. ������ ���� 314.
	ColorTypeNumber_P_Section = ReadIntegerDataType(P_SectionVector, i, 'P');  // ������ Entity Type Number
	// ��������� ������ �������� - Entity Type Number (������ ���� 314)
	if (ColorTypeNumber_P_Section != 314) {
		string txt = "\n������ � �������� �����:\n� ������� Parameter Data Section �������� Entity Type Number. � ������ " + to_string(ColorField2_ParameterDataLine) +
			" ������� \n�������� " + to_string(ColorTypeNumber_P_Section) + " (������ ���� 314).";
		PrintErrorAndExit(txt);
	}
	// ������ ����������� ����� ������� ��������� � Parameter Data Section (�.�. ����� Entity Type Number)
	if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
		string txt = "������ � �������� �����! � ������� Parameter Data Section � ������ " + to_string(ColorField2_ParameterDataLine) + ".\n";
		txt = txt + "�������� ����������� ����� ������� ��������� (������ ���� ������  " + Global_GlobalParameters.GP1_ParameterDelimiter + " ).";
		PrintErrorAndExit(txt);
	}
	// ������ ������ �������� - First color coordinate (red)
	ColorCoordinate1_RED = ReadRealDataType(P_SectionVector, i, 'P');
	// ��������� ������ �������� - First color coordinate (red) (������ ���� �� 0 �� 100 ������������)
	if (ColorCoordinate1_RED < 0. || ColorCoordinate1_RED > 100.) {
		string txt = "\n������ � �������� �����:\n� ������� Parameter Data Section, � ������ " + to_string(ColorField2_ParameterDataLine) +
			" ������ �������� = " + to_string(ColorCoordinate1_RED) + "\n(������ ���� �� 0 �� 100 ������������).";
		PrintErrorAndExit(txt);
	}
	// ������ ����������� ����� ������� ��������� � Parameter Data Section, �.�. ����� First color coordinate (red)
	if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
		string txt = "������! � ������� Parameter Data Section � ������ " + to_string(ColorField2_ParameterDataLine) + ".\n";
		txt = txt + "�������� ����������� ����� ������� ��������� (������ ���� ������  " + Global_GlobalParameters.GP1_ParameterDelimiter + " ).";
		PrintErrorAndExit(txt);
	}
	// ������ ������ �������� - Second color coordinate (green)
	ColorCoordinate2_GREEN = ReadRealDataType(P_SectionVector, i, 'P');
	// ��������� ������ �������� - Second color coordinate (green) (������ ���� �� 0 �� 100 ������������)
	if (ColorCoordinate2_GREEN < 0. || ColorCoordinate2_GREEN > 100.) {
		string txt = "\n������ � �������� �����:\n� ������� Parameter Data Section, � ������ " + to_string(ColorField2_ParameterDataLine) +
			" ������ �������� = " + to_string(ColorCoordinate2_GREEN) + "\n(������ ���� �� 0 �� 100 ������������).";
		PrintErrorAndExit(txt);
	}
	// ������ ����������� ����� �������� ��������� � Parameter Data Section, �.�. ����� Second color coordinate (green)
	if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
		string txt = "������ � �������� �����! � ������� Parameter Data Section � ������ " + to_string(ColorField2_ParameterDataLine) + ".\n";
		txt = txt + "�������� ����������� ����� �������� ��������� (������ ���� ������  " + Global_GlobalParameters.GP1_ParameterDelimiter + " ).";
		PrintErrorAndExit(txt);
	}
	// ������ �������� �������� - Third color coordinate (blue)
	ColorCoordinate3_BLUE = ReadRealDataType(P_SectionVector, i, 'P');
	// ��������� �������� �������� - Third color coordinate (blue) (������ ���� �� 0 �� 100 ������������)
	if (ColorCoordinate3_BLUE < 0. || ColorCoordinate3_BLUE > 100.) {
		string txt = "\n������ � �������� �����:\n� ������� Parameter Data Section, � ������ " + to_string(ColorField2_ParameterDataLine) +
			" �������� �������� = " + to_string(ColorCoordinate3_BLUE) + "\n(������ ���� �� 0 �� 100 ������������).";
		PrintErrorAndExit(txt);
	}
	// ������ ����������� ����� ��������� ��������� � Parameter Data Section, �.�. ����� Third color coordinate (blue)
	if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
		string txt = "������! � ������� Parameter Data Section � ������ " + to_string(ColorField2_ParameterDataLine) + ".\n";
		txt = txt + "�������� ����������� ����� ��������� ��������� (������ ���� ������  " + Global_GlobalParameters.GP1_ParameterDelimiter + " ).";
		PrintErrorAndExit(txt);
	}
	// -- ����������� �������� RGB --
	// ����������� Color_R
	static_cast<int>(255 * ColorCoordinate1_RED / 100) == static_cast<int>(255 * ColorCoordinate1_RED / 100 + 0.5) ?
		Color_R = static_cast<int>(255 * ColorCoordinate1_RED / 100) : Color_R = static_cast<int>(255 * ColorCoordinate1_RED / 100 + 0.5);
	// ����������� Color_G
	static_cast<int>(255 * ColorCoordinate2_GREEN / 100) == static_cast<int>(255 * ColorCoordinate2_GREEN / 100 + 0.5) ?
		Color_G = static_cast<int>(255 * ColorCoordinate2_GREEN / 100) : Color_G = static_cast<int>(255 * ColorCoordinate2_GREEN / 100 + 0.5);
	// ����������� Color_B
	static_cast<int>(255 * ColorCoordinate3_BLUE / 100) == static_cast<int>(255 * ColorCoordinate3_BLUE / 100 + 0.5) ?
		Color_B = static_cast<int>(255 * ColorCoordinate3_BLUE / 100) : Color_B = static_cast<int>(255 * ColorCoordinate3_BLUE / 100 + 0.5);
	// --- ���������� �������� �����  ---
	if (Color_R == 0 && Color_G == 0 && Color_B == 0) return 1;  // RGB 0,0,0 - ���� 1 - ׸����
	if (Color_R == 255 && Color_G == 0 && Color_B == 0) return 2;  // RGB 255,0,0 - ���� 2 - �������
	if (Color_R == 0 && Color_G == 255 && Color_B == 0) return 3;  // RGB 0,255,0 - ���� 3 - ������
	if (Color_R == 0 && Color_G == 0 && Color_B == 255) return 4;  // RGB 0,0,255 - ���� 4 - �����
	if (Color_R == 255 && Color_G == 255 && Color_B == 0) return 5;  // RGB 255,255,0 - ���� 5 - Ƹ����
	if (Color_R == 255 && Color_G == 0 && Color_B == 255) return 6;  // RGB 255,0,255 - ���� 6 - Magenta (���������)
	if (Color_R == 0 && Color_G == 255 && Color_B == 255) return 7;  // RGB 0,255,255 - ���� 7 - Cyan (���������)
	if (Color_R == 255 && Color_G == 255 && Color_B == 255) return 8;  // RGB 255,255,255 - ���� 8 - �����
	else return 1000000000 + Color_R * 1000000 + Color_G * 1000 + Color_B;
}   // ����� ������� ReadEntityColor()
//
//
// ������ "�������" (Line Entity - Type 110) � Directory Entry Section �� ������ D_Sect_Str_Number.
// ������������ � ���������� ���������� � ������ Global_Entity110_Line_Vector. 
void ReadEntity110_Line(const vector<string>& D_SectionVector, const vector<string>& P_SectionVector, const int D_Sect_Str_Number)
{
	Entity110_Line TMPEntity110_Line;  // ��������� ��������� ��� ������ ������ �� �������. 
	int Field2_ParameterDataLine = ReadIntegerIn_D_Section(D_SectionVector, D_Sect_Str_Number, 2);  // ����� � ���� 2 (Parameter Data Pointer)
		// stoi(D_SectionVector[D_Sect_Str_Number-1].substr(8, 8)); // ����� � ���� 2 (Parameter Data Pointer)  // �������
	int Field7_PointerTransformMatrix = ReadIntegerIn_D_Section(D_SectionVector, D_Sect_Str_Number, 7);  // ����� � ���� 7 (Pointer to the Directory Entry of a Transformation Matrix (Type 124))
		// stoi(D_SectionVector[D_Sect_Str_Number-1].substr(48, 8));  // ����� � ���� 7 (Pointer to the Directory Entry of a Transformation Matrix (Type 124))    �������
	int Field15_FormNumber;  //  ���� 15 � Directory Entry. 0 - �������. 1 - ��� � ������� � (X1,Y1,Z1). 2 - ����������� ������
	int TypeNumber{ 0 };  // ����� ������� (������ ���� 110 "Line") - ��� �������� ��������� 1 � Parameter Data Section
	TMPEntity110_Line.D_Section_String_Number = D_Sect_Str_Number;
	TMPEntity110_Line.P_Section_String_Number = Field2_ParameterDataLine;
	// --- ������ ���������� ������� � Parameter Data Section ---
	{
		int i = (Field2_ParameterDataLine - 1) * Global_PSectionLineLength;   // ������ ��������� ������� � Parameter Data Section
		// ������ ������ �������� � Parameter Data Section - Entity Type Number. ������ ���� 110.
		TypeNumber = ReadIntegerDataType(P_SectionVector, i, 'P');
		// ��������� Entity Type Number (������ ���� 110)
		if (TypeNumber != Global_Entity110LineNumber) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section �������� Entity Type Number.\n� ������ " + to_string(Field2_ParameterDataLine) +
				" ������� �������� " + to_string(TypeNumber) + " (������ ���� 110).";
			PrintErrorAndExit(txt);
		}
		// ������ ����������� ����� ������� ��������� � Parameter Data Section (�.�. ����� Entity Type Number)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ������� ���������\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������ �������� � Parameter Data Section - X1.
		TMPEntity110_Line.X1origin = ReadRealDataType(P_SectionVector, i, 'P');
		// ������ ����������� ����� ������� ��������� � Parameter Data Section (�.�. ����� X1)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ������� ���������\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������ �������� � Parameter Data Section - Y1.
		TMPEntity110_Line.Y1origin = ReadRealDataType(P_SectionVector, i, 'P');
		// ������ ����������� ����� �������� ��������� � Parameter Data Section (�.�. ����� Y1)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� �������� ���������\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ �������� �������� � Parameter Data Section - Z1.
		TMPEntity110_Line.Z1origin = ReadRealDataType(P_SectionVector, i, 'P');
		// ������ ����������� ����� ��������� ��������� � Parameter Data Section (�.�. ����� Z1)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ��������� ���������\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ����� �������� � Parameter Data Section - X2.
		TMPEntity110_Line.X2origin = ReadRealDataType(P_SectionVector, i, 'P');
		// ������ ����������� ����� ������ ��������� � Parameter Data Section (�.�. ����� X2)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ������ ���������\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������ �������� � Parameter Data Section - Y2.
		TMPEntity110_Line.Y2origin = ReadRealDataType(P_SectionVector, i, 'P');
		// ������ ����������� ����� ������� ��������� � Parameter Data Section (�.�. ����� Y2)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ������� ���������\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������� �������� � Parameter Data Section - Z2.
		TMPEntity110_Line.Z2origin = ReadRealDataType(P_SectionVector, i, 'P');
	}
	// ������������ ����� �������
	TMPEntity110_Line.Length = sqrt((TMPEntity110_Line.X2origin - TMPEntity110_Line.X1origin) * (TMPEntity110_Line.X2origin - TMPEntity110_Line.X1origin) +
		(TMPEntity110_Line.Y2origin - TMPEntity110_Line.Y1origin) * (TMPEntity110_Line.Y2origin - TMPEntity110_Line.Y1origin) +
		(TMPEntity110_Line.Z2origin - TMPEntity110_Line.Z1origin) * (TMPEntity110_Line.Z2origin - TMPEntity110_Line.Z1origin));
	// ������ ���� 15 - Form Number. ��������� ������������ �������� - ������ ���� 0, 1 ��� 2.
	Field15_FormNumber = ReadIntegerIn_D_Section(D_SectionVector, D_Sect_Str_Number, 15);
	//  stoi(D_SectionVector[D_Sect_Str_Number].substr(32, 8));        �������
	if (Field15_FormNumber < 0 || Field15_FormNumber > 2) {
		string txt = "������ � �������� �����!\n� ������� Directory Entry �������� �������� � ���� 15 �������� � ������ " + to_string(D_Sect_Str_Number) +
			".\n������� �������� " + to_string(Field15_FormNumber) + " (�������� ������ ���� 0, 1 ��� 2).";
		PrintErrorAndExit(txt);
	}
	TMPEntity110_Line.FormNumber = Field15_FormNumber;
	// --- ������ ���� ������� ---
	TMPEntity110_Line.Color = ReadEntityColor(D_SectionVector, P_SectionVector, D_Sect_Str_Number);
	// --- ������ ������� ������������� (��������� ������� � ��� �������� � �� ������� (���� ����� ����))---
	ReadEntity124_TrMatrix(TMPEntity110_Line.TransformationMatrixVector, D_SectionVector, P_SectionVector, Field7_PointerTransformMatrix);
	// ����������� ���������� ����� ����� ������� �������������
	TransformPointByTransfMatrix(TMPEntity110_Line.X1origin, TMPEntity110_Line.Y1origin, TMPEntity110_Line.Z1origin,
		TMPEntity110_Line.X1, TMPEntity110_Line.Y1, TMPEntity110_Line.Z1, TMPEntity110_Line.TransformationMatrixVector);
	TransformPointByTransfMatrix(TMPEntity110_Line.X2origin, TMPEntity110_Line.Y2origin, TMPEntity110_Line.Z2origin,
		TMPEntity110_Line.X2, TMPEntity110_Line.Y2, TMPEntity110_Line.Z2, TMPEntity110_Line.TransformationMatrixVector);
	//  ��������� ����������� ������� � ������ ��������
	Global_Entity110_Line_Vector.push_back(TMPEntity110_Line);
}  // ����� ������� ReadEntity110_Line()
//
//
// ������ ������� ������������� (Transformation Matrix Entity - Type 124) � Directory Entry Section, �������
// ��������� �� ������ Entity124_D_Sect_Str_Number, � ����� ������ ��� �������� � �� ������� (���� ����� ����). ���������� ��������� � TransformationMatrixVector.
void ReadEntity124_TrMatrix(vector<Entity124_TransformationMatrix>& TransformationMatrixVector, const vector<string>& D_SectionVector,
	const vector<string>& P_SectionVector, int Entity124_D_Sect_Str_Number)
{
	if (Entity124_D_Sect_Str_Number == 0) return;
	int EntityNumber;  // ���� 1 � Directory Entry, ��� ������ �������� � Parameter Data - ��� ��������� � ���������� ��������� (Global_Entity124TransMatrixNumber)
	int Field2_ParameterDataLine;  // ����� � ���� 2 - ��������� (Pointer) �� ������ � Parameter Data
	int Field7_TrMatrixPointer;  // ����� � ���� 7 (��������� �� ��������� Transformation Matrix). 
	int Field15_FormNumber;  // ������ ���� 0 ��� 1
	Entity124_TransformationMatrix TmpEntity124_TrMatrix;  // ��������� ��������� ��� ������ ������� �������������. ����� ���������� ����������� � TransformationMatrixVector
	// ������ � ��������� ���� 1 � Directory Entry (EntityNumber). ������ ���� 124
	EntityNumber = ReadIntegerIn_D_Section(D_SectionVector, Entity124_D_Sect_Str_Number, 1);
	// stoi(D_SectionVector[Entity124_D_Sect_Str_Number - 1].substr(0, 8));     �������
	if (EntityNumber != 124) {
		string txt = "������ � �������� �����!\n� ������� Directory Entry �������� �������� � ���� 1 � ������ " + to_string(Entity124_D_Sect_Str_Number) +
			".\n������� �������� " + to_string(EntityNumber) + " (������ ���� 124).";
		PrintErrorAndExit(txt);
	}
	// ������ ���� 2 - Parameter Data Pointer. ��������� ������������ ��������: ������ ���� ������ 0, �� ������ ������� P_SectionVector.size()
	Field2_ParameterDataLine = ReadIntegerIn_D_Section(D_SectionVector, Entity124_D_Sect_Str_Number, 2);
	// stoi(D_SectionVector[Entity124_D_Sect_Str_Number - 1].substr(8, 8));    �������
	if (Field2_ParameterDataLine <= 0) {
		string txt = "������ � �������� �����!\n� ������� Directory Entry �������� �������� � ���� 2 �������� � ������ " + to_string(Entity124_D_Sect_Str_Number) +
			".\n������� �������� " + to_string(Field2_ParameterDataLine) + " (�������� ������ ���� ������ ����).";
		PrintErrorAndExit(txt);
	}
	else if (Field2_ParameterDataLine > P_SectionVector.size()) {
		string txt = "������ � �������� �����!\n� ������� Directory Entry �������� �������� � ���� 2 �������� � ������ " + to_string(Entity124_D_Sect_Str_Number) +
			".\n������� �������� " + to_string(Field2_ParameterDataLine) + " \n(�������� ������ ���� ������ ������� Parameter Data Section).";
		PrintErrorAndExit(txt);
	}
	TmpEntity124_TrMatrix.D_Section_String_Number = Entity124_D_Sect_Str_Number;
	TmpEntity124_TrMatrix.P_Section_String_Number = Field2_ParameterDataLine;
	// ������ ���� 7 - ��������� �� ��������� Transformation Matrix. ��������� ������������ ��������.
	Field7_TrMatrixPointer = ReadIntegerIn_D_Section(D_SectionVector, Entity124_D_Sect_Str_Number, 7);
	// stoi(D_SectionVector[Entity124_D_Sect_Str_Number - 1].substr(48, 8));      �������
	if (Field7_TrMatrixPointer < 0) {
		string txt = "������ � �������� �����!\n� ������� Directory Entry �������� �������� � ���� 7 �������� � ������ " + to_string(Entity124_D_Sect_Str_Number) +
			".\n������� �������� " + to_string(Field7_TrMatrixPointer) + " (�������� ������ ���� ������ ��� ����� ����).";
		PrintErrorAndExit(txt);
	}
	else if (Field7_TrMatrixPointer == Entity124_D_Sect_Str_Number) {
		string txt = "������ � �������� �����!\n� ������� Directory Entry �������� �������� � ���� 7 �������� � ������ " + to_string(Entity124_D_Sect_Str_Number) +
			".\n������� �������� " + to_string(Field7_TrMatrixPointer) + " (�������� �� ������ ���� ����� ������ ����������� ������).";
		PrintErrorAndExit(txt);
	}
	else if (Field7_TrMatrixPointer > D_SectionVector.size()) {
		string txt = "������ � �������� �����!\n� ������� Directory Entry �������� �������� � ���� 7 �������� � ������ " + to_string(Entity124_D_Sect_Str_Number) +
			".\n������� �������� " + to_string(Field7_TrMatrixPointer) + " \n(�������� ������ ���� ������ ������� Directory Entry Section).";
		PrintErrorAndExit(txt);
	}
	// ������ ���� 15 - Form Number. ��������� ������������ �������� - ������ ���� 1 ��� 2.
	Field15_FormNumber = ReadIntegerIn_D_Section(D_SectionVector, Entity124_D_Sect_Str_Number, 15);
	if (Field15_FormNumber < 0 || Field15_FormNumber > 1) {   // ��������� ������������ �������� - ������ ���� 1 ��� 2. ����� - ������ � ����� �� ���������
		string txt = "������ � �������� �����!\n� ������� Directory Entry �������� �������� � ���� 15 �������� � ������ " + to_string(Entity124_D_Sect_Str_Number) +
			".\n������� �������� " + to_string(Field15_FormNumber) + " (�������� ������ ���� 0 ��� 1).";
		PrintErrorAndExit(txt);
	}
	if (Field15_FormNumber == 1) {
		string txt = "��������!\n� IGES-����� ���������� left-handed ������� �������������.\n";
		txt = txt + "� Directory Entry Section, ������� ������������� Entity Type Number 124\n�� ������ " + to_string(Entity124_D_Sect_Str_Number) +
			", �������� � ���� 15 ����� 1 (������ ���� = 0).\n�������� ������������ �������� ���������.";
		PrintErrorAndExit(txt);
	}
	TmpEntity124_TrMatrix.FormNumber = Field15_FormNumber;
	// --- ������ ���������� � ������� ������������� � Parameter Data ---
	{
		int i = (Field2_ParameterDataLine - 1) * Global_PSectionLineLength;   // ������ ��������� ������� � Parameter Data Section
		// ������ � ��������� ������ ���� � Parameter Data Section (Entity Type Number). ������ ���� 124	
		EntityNumber = ReadIntegerDataType(P_SectionVector, i, 'P');
		if (EntityNumber != 124) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data �������� �������� � ������ ���� � ������ " + to_string(Field2_ParameterDataLine) +
				".\n������� �������� " + to_string(EntityNumber) + " (������ ���� 124).";
			PrintErrorAndExit(txt);
		}
		// ������ ����������� ����� ������� ���� � Parameter Data Section (�.�. ����� Entity Type Number)
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ������� ����\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������ �������� � Parameter Data Section, �.�. �������� ������ 1 (R11)
		TmpEntity124_TrMatrix.R11 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ������� ���� - ����� R11\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������ �������� � Parameter Data Section, �.�. �������� ������ 2 (R12)
		TmpEntity124_TrMatrix.R12 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� �������� ���� - ����� R12\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ �������� �������� � Parameter Data Section, �.�. �������� ������ 3 (R13)
		TmpEntity124_TrMatrix.R13 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ��������� ���� - ����� R13\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ����� �������� � Parameter Data Section, �.�. �������� ������ 4 (T1)
		TmpEntity124_TrMatrix.T1 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ������ ���� - ����� T1\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������ �������� � Parameter Data Section, �.�. �������� ������ 5 (R21)
		TmpEntity124_TrMatrix.R21 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ������� ���� - ����� R21\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������� �������� � Parameter Data Section, �.�. �������� ������ 6 (R22)
		TmpEntity124_TrMatrix.R22 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� �������� ���� - ����� R22\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������� �������� � Parameter Data Section, �.�. �������� ������ 7 (R23)
		TmpEntity124_TrMatrix.R23 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� �������� ���� - ����� R23\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������� �������� � Parameter Data Section, �.�. �������� ������ 8 (T2)
		TmpEntity124_TrMatrix.T2 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� �������� ���� - ����� T2\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������� �������� � Parameter Data Section, �.�. �������� ������ 9 (R31)
		TmpEntity124_TrMatrix.R31 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ��c����� ���� - ����� R31\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ������������ �������� � Parameter Data Section, �.�. �������� ������ 10 (R32)
		TmpEntity124_TrMatrix.R32 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ������������� ���� - ����� R32\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ����������� �������� � Parameter Data Section, �.�. �������� ������ 11 (R33)
		TmpEntity124_TrMatrix.R33 = ReadRealDataType(P_SectionVector, i, 'P');
		if (ReadDelimiters(P_SectionVector, i, 'P') != Global_GlobalParameters.GP1_ParameterDelimiter) {
			string txt = "������ � �������� �����!\n� ������� Parameter Data Section � ������ " + to_string(Field2_ParameterDataLine) +
				" �������� ����������� ����� ������������ ���� - ����� R33\n(������ ���� ������  \'" + Global_GlobalParameters.GP1_ParameterDelimiter + "\').";
			PrintErrorAndExit(txt);
		}
		// ������ ����������� �������� � Parameter Data Section, �.�. �������� ������ 12 (T3)
		TmpEntity124_TrMatrix.T3 = ReadRealDataType(P_SectionVector, i, 'P');
	}
	TransformationMatrixVector.push_back(TmpEntity124_TrMatrix);   // 
	// �������� ��� ������� ���������� ���� ���� ��������� �������
	if (Field7_TrMatrixPointer != 0) ReadEntity124_TrMatrix(TransformationMatrixVector, D_SectionVector, P_SectionVector, Field7_TrMatrixPointer);
}    //  ����� �������  ReadEntity124_TrMatrix()
//
// 
// ����������� ���������� ����� Xorigin, Yorigin, Zorigin (������ � IGES-�����) � ���������� X1, Y1, Z1 ��������� ������� 
// ��������������  TransformationMatrixVector. ������� ��������� �����������.
void TransformPointByTransfMatrix(const double& Xorigin, const double& Yorigin, const double& Zorigin, double& X1, double& Y1, double& Z1, vector<Entity124_TransformationMatrix>& TransformationMatrixVector)
{
	if (TransformationMatrixVector.empty()) {   //  ���� ������ ������������� ���, �� X1, Y1, Z1 ����� Xorigin, Yorigin, Zorigin
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




