block LCO_cellCalculator
  //input ports
  Modelica.Blocks.Interfaces.RealInput i annotation(
    Placement(transformation(origin = {-106, 66}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-106, 66}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput T annotation(
    Placement(transformation(origin = {-106, -42}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-106, -42}, extent = {{-20, -20}, {20, 20}})));
  
  //output ports
  Modelica.Blocks.Interfaces.RealOutput SOC annotation(
    Placement(transformation(origin = {106, 78}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {106, 78}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput cycles annotation(
    Placement(transformation(origin = {106, -40}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {106, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput capacity annotation(
    Placement(transformation(origin = {106, 10}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {106, 10}, extent = {{-10, -10}, {10, 10}})));
  
  parameter Real b = 0.453;
  parameter Real initSOC;
  parameter Real Q0;
  Real CF "Capacity fade (%)";
  Real RUL "Remaining Useful Life (%)";
  Real DoD "Depth of Discharge (%)";
  Real mSOC "mean SOC (%)";
  Real maxSOC(start=initSOC);
  Real minSOC(start=initSOC);
  Modelica.Blocks.Math.Abs abs1 annotation(
    Placement(transformation(origin = {-20, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Gain gain1(k = 0.5)  annotation(
    Placement(transformation(origin = {10, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Division division1 annotation(
    Placement(transformation(origin = {46, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integrator1 annotation(
    Placement(transformation(origin = {76, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Division division2 annotation(
    Placement(transformation(origin = {40, 78}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.LimIntegrator integrator2(outMax = 1 - (1e-6), outMin = 1e-6, y_start = initSOC)  annotation(
    Placement(transformation(origin = {74, 78}, extent = {{-10, -10}, {10, 10}})));
algorithm
  maxSOC := max(delay(maxSOC,1),SOC);
  minSOC := min(delay(minSOC,1),SOC);
  
equation
  CF = 1 - (capacity/Q0);
  RUL = 1 - (CF/0.2);
  capacity = (Q0/100)*(100-(3.25*mSOC*(1+3.25*DoD-2.25*(DoD^2)))*((cycles)^b));
  DoD = maxSOC - minSOC;
  mSOC = (DoD/2) + minSOC;
  division1.u2 = capacity;
  division2.u2 = capacity;
  connect(abs1.y, gain1.u) annotation(
    Line(points = {{-9, -20}, {-3, -20}}, color = {0, 0, 127}));
  connect(division1.u1, gain1.y) annotation(
    Line(points = {{34, -34}, {28, -34}, {28, -20}, {22, -20}}, color = {0, 0, 127}));
  connect(division1.y, integrator1.u) annotation(
    Line(points = {{58, -40}, {64, -40}}, color = {0, 0, 127}));
  connect(integrator1.y, cycles) annotation(
    Line(points = {{88, -40}, {106, -40}}, color = {0, 0, 127}));
  connect(i, abs1.u) annotation(
    Line(points = {{-106, 66}, {-56, 66}, {-56, -20}, {-32, -20}}, color = {0, 0, 127}));
  connect(division2.y, integrator2.u) annotation(
    Line(points = {{52, 78}, {62, 78}}, color = {0, 0, 127}));
  connect(division2.u1, i) annotation(
    Line(points = {{28, 84}, {4, 84}, {4, 66}, {-106, 66}}, color = {0, 0, 127}));
  connect(integrator2.y, SOC) annotation(
    Line(points = {{86, 78}, {106, 78}}, color = {0, 0, 127}));

annotation(
    uses(Modelica(version = "4.0.0")));
end LCO_cellCalculator;
