model IMECE2025v102
  Modelica.Electrical.Analog.Sensors.MultiSensor fcmeter annotation(
    Placement(visible = true, transformation(origin = {-14, 34}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  PhysicsBasedPredictionRULv101.detailedFC fc(N = 300, N_m = 8)  annotation(
    Placement(visible = true, transformation(origin = {20, 32}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PhysicsBasedPredictionRULv101.simpleTank tank(Cv = 0.6)  annotation(
    Placement(visible = true, transformation(origin = {76, 32}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain annotation(
    Placement(visible = true, transformation(origin = {48, 42}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
  PhysicsBasedPredictionRULv101.flightCycle cycle(cruiseP(displayUnit = "kW") = 154000, cruiseTime = 2700, maxP(displayUnit = "kW") = 212000, maxRamp = 30, maxTime = 180, sigma = 2554.4, taxiP(displayUnit = "kW") = 41000, taxiRamp = 60, taxiTime = 592)  annotation(
    Placement(visible = true, transformation(origin = {-76, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 3e5)  annotation(
    Placement(visible = true, transformation(origin = {47, 33}, extent = {{-3, -3}, {3, 3}}, rotation = 180)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
    Placement(visible = true, transformation(origin = {30, 8}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  Modelica.Electrical.Analog.Sources.SignalCurrent fcLoad annotation(
    Placement(visible = true, transformation(origin = {-44, 32}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Electrical.Analog.Sensors.MultiSensor batmeter annotation(
    Placement(visible = true, transformation(origin = {-14, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.SignalCurrent batLoad annotation(
    Placement(visible = true, transformation(origin = {-44, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Logical.TerminateSimulation terminateSimulation(condition = stack.cellCalculator.RUL <= 0)  annotation(
    Placement(visible = true, transformation(origin = {-10, 62}, extent = {{-40, -4}, {40, 4}}, rotation = 0)));
  PhysicsBasedPredictionRULv101.LCO_Stack stack(cellData = batParams, Ns = 168, Np = 48, initSOC = 1)  annotation(
    Placement(transformation(origin = {20, -18}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  PhysicsBasedPredictionRULv101.CellParameters batParams(Qnom = 10800, Ri = 0.0162421792644079, OCVtableOnFile = true,final OCVtableName = "18650SOCvOVC",final OCVfileName = "C:\\Users\\coopern2\\Downloads\\30Q_SOCOCV.csv")  annotation(
    Placement(transformation(origin = {50, -16}, extent = {{-10, -10}, {10, 10}})));
equation
  fcLoad.i = max(cycle.fcLoad/fcmeter.v, 1e-6);
  batLoad.i = cycle.batLoad/batmeter.v;
  tank.limIntegrator.reset = cycle.refuel;
  connect(gain.y, fc.q_air) annotation(
    Line(points = {{44, 42}, {30, 42}}, color = {0, 0, 127}));
  connect(tank.q_fuel, gain.u) annotation(
    Line(points = {{66, 36}, {60, 36}, {60, 42}, {52, 42}}, color = {0, 0, 127}));
  connect(tank.q_fuel, fc.q_fuel) annotation(
    Line(points = {{66, 36}, {60, 36}, {60, 26}, {30, 26}}, color = {0, 0, 127}));
  connect(tank.p_fuel, fc.p_fuel) annotation(
    Line(points = {{66, 24}, {30, 24}}, color = {0, 0, 127}));
  connect(const.y, fc.p_air) annotation(
    Line(points = {{44, 33}, {36, 33}, {36, 38}, {30, 38}}, color = {0, 0, 127}));
  connect(fcmeter.nv, fc.n) annotation(
    Line(points = {{-14, 24}, {-14, 12}, {20, 12}, {20, 22}}, color = {0, 0, 255}));
  connect(fcmeter.pv, fc.p) annotation(
    Line(points = {{-14, 44}, {-14, 52}, {20, 52}, {20, 42}}, color = {0, 0, 255}));
  connect(fcmeter.pc, fc.p) annotation(
    Line(points = {{-4, 34}, {2, 34}, {2, 52}, {20, 52}, {20, 42}}, color = {0, 0, 255}));
  connect(fcLoad.p, fcmeter.nc) annotation(
    Line(points = {{-44, 42}, {-44, 52}, {-30, 52}, {-30, 34}, {-24, 34}}, color = {0, 0, 255}));
  connect(fcLoad.n, fc.n) annotation(
    Line(points = {{-44, 22}, {-44, 12}, {20, 12}, {20, 22}}, color = {0, 0, 255}));
  connect(ground.p, fc.n) annotation(
    Line(points = {{20, 8}, {20, 22}}, color = {0, 0, 255}));
  connect(batLoad.n, batmeter.pc) annotation(
    Line(points = {{-44, -6}, {-44, 4}, {-28, 4}, {-28, -16}, {-24, -16}}, color = {0, 0, 255}));
  connect(batmeter.pv, batLoad.n) annotation(
    Line(points = {{-14, -6}, {-14, 4}, {-44, 4}, {-44, -6}}, color = {0, 0, 255}));
  connect(batmeter.nv, batLoad.p) annotation(
    Line(points = {{-14, -26}, {-14, -36}, {-44, -36}, {-44, -26}}, color = {0, 0, 255}));
  connect(stack.p, ground.p) annotation(
    Line(points = {{20, -8}, {20, 8}}, color = {0, 0, 255}));
  connect(stack.p, batmeter.nc) annotation(
    Line(points = {{20, -8}, {20, 4}, {6, 4}, {6, -16}, {-4, -16}}, color = {0, 0, 255}));
  connect(stack.n, batLoad.p) annotation(
    Line(points = {{20, -28}, {20, -36}, {-44, -36}, {-44, -26}}, color = {0, 0, 255}));
  annotation(
    uses(Modelica(version = "4.0.0")));
end IMECE2025v102;
