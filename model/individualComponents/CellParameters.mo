record CellParameters
  extends Modelica.Electrical.Batteries.Icons.BaseCellRecord;
  parameter Modelica.Units.SI.ElectricCharge Qnom(displayUnit="A.h")
    "Nominal (maximum) charge";
  parameter Modelica.Units.SI.Resistance Ri "Total inner resistance (= OCVmax/Isc)";
  parameter Modelica.Units.SI.Temperature T_ref=293.15 "Reference temperature";
  parameter Modelica.Units.SI.LinearTemperatureCoefficient alpha=0 "Temperature coefficient of resistance at T_ref";
  parameter Modelica.Units.SI.Current Idis=0 "Self-discharge current at SOC = SOCmax"
    annotation(Evaluate=true);
  parameter Modelica.Units.SI.Resistance R0=Ri
    "Inner resistance without parallel C";
  parameter Boolean OCVtableOnFile = false "true, if OCV table is defined on file or in function usertab";
  parameter Real OCVtable[:,2] = [0,2.7; 0.0085, 3.313; 0.05, 3.35; 0.1, 3.49; 0.2, 3.55; 0.4, 3.65; 0.6, 3.75; 0.75, 3.85; 0.9, 4; 1, 4.2] "SOC = first column, OCV = second column";
  parameter String OCVtableName = "NoName" "OCV table name on file or in function usertab";
  parameter String OCVfileName = "NoName" "file where OCV matrix is stored";
end CellParameters;
