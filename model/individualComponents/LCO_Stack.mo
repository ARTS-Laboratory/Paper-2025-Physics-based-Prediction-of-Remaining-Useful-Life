model LCO_Stack "Battery with open-circuit voltage dependent on state of charge, self-discharge and inner resistance"
  extends Modelica.Electrical.Batteries.Icons.BatteryIcon(final displaySOC = cellCalculator.SOC);
  parameter Integer Ns(final min = 1) = 1 "Number of serial connected cells";
  parameter Integer Np(final min = 1) = 1 "Number of parallel connected cells";
  parameter Real SOCtolerance = 1e-9 "Tolerance to detect depleted of overcharged battery" annotation(
    Dialog(tab = "Advanced"));
  parameter Real initSOC;
  
  extends Modelica.Electrical.Analog.Interfaces.TwoPin;
  Modelica.Units.SI.Current i = p.i "Current into the battery";
  Modelica.Units.SI.Power power = v*i "Power to the battery";
  //output Real SOC(start = 1) = cellCalculator.SOC "State of charge" annotation(
    //Dialog(showStartAttribute = true));
  Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor annotation(
    Placement(transformation(extent = {{-90, 10}, {-70, -10}})));
  Modelica.Blocks.Math.Gain gainV(final k = Ns) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-40, 30})));
  Modelica.Electrical.Analog.Sources.SignalVoltage ocv annotation(
    Placement(transformation(extent = {{-50, -10}, {-30, 10}})));
  Modelica.Electrical.Analog.Basic.Conductor selfDischarge(final G = Np*cellData.Idis/(Ns), T_ref = 293.15, final useHeatPort = true) annotation(
    Placement(transformation(origin = {8, 0}, extent = {{-70, -30}, {-50, -10}})));
  Modelica.Electrical.Analog.Basic.Resistor r0(final T_ref = cellData.T_ref, final alpha = cellData.alpha, final useHeatPort = true) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  extends Modelica.Electrical.Analog.Interfaces.PartialConditionalHeatPort;
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temp annotation(
    Placement(transformation(origin = {-20, -80}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Tables.CombiTable1Dv ocv_soc(final tableOnFile = cellData.OCVtableOnFile, final table = cellData.OCVtable, final tableName = cellData.OCVtableName, final fileName = cellData.OCVfileName, final smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
    Placement(transformation(origin = {-60, 58}, extent = {{-10, -10}, {10, 10}})));
  CellParameters cellData annotation(
    Placement(transformation(origin = {24, 72}, extent = {{-10, -10}, {10, 10}})));
  LCO_cellCalculator cellCalculator(initSOC = initSOC, Q0 = cellData.Qnom)  annotation(
    Placement(transformation(origin = {-74, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
initial equation
//cellCalculator.SOC = initSOC;

equation
  connect(gainV.y, ocv.v) annotation(
    Line(points = {{-40, 19}, {-40, 12}}, color = {0, 0, 127}));
  connect(currentSensor.n, ocv.p) annotation(
    Line(points = {{-70, 0}, {-50, 0}}, color = {0, 0, 255}));
  connect(p, currentSensor.p) annotation(
    Line(points = {{-100, 0}, {-90, 0}}, color = {0, 0, 255}));
  connect(ocv.n, r0.p) annotation(
    Line(points = {{-30, 0}, {-10, 0}}, color = {0, 0, 255}));
  connect(currentSensor.p, selfDischarge.p) annotation(
    Line(points = {{-90, 0}, {-90, -20}, {-62, -20}}, color = {0, 0, 255}));
  connect(ocv.n, selfDischarge.n) annotation(
    Line(points = {{-30, 0}, {-30, -20}, {-42, -20}}, color = {0, 0, 255}));
  connect(selfDischarge.heatPort, internalHeatPort) annotation(
    Line(points = {{-52, -30}, {-52, -40}, {0, -40}, {0, -80}}, color = {191, 0, 0}));
  connect(internalHeatPort, r0.heatPort) annotation(
    Line(points = {{0, -80}, {0, -10}}, color = {191, 0, 0}));
  connect(r0.n, n) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 255}));
  connect(internalHeatPort, temp.port) annotation(
    Line(points = {{0, -80}, {-10, -80}}, color = {191, 0, 0}));
  connect(ocv_soc.y[1], gainV.u) annotation(
    Line(points = {{-48, 58}, {-40, 58}, {-40, 42}}, color = {0, 0, 127}));
  connect(currentSensor.i, cellCalculator.i) annotation(
    Line(points = {{-80, 12}, {-80, 20}}, color = {0, 0, 127}));
  connect(temp.T, cellCalculator.T) annotation(
    Line(points = {{-30, -80}, {-70, -80}, {-70, 20}}, color = {0, 0, 127}));
  connect(cellCalculator.SOC, ocv_soc.u[1]) annotation(
    Line(points = {{-82, 40}, {-82, 58}, {-72, 58}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "4.0.0")),
    Diagram);
end LCO_Stack;
