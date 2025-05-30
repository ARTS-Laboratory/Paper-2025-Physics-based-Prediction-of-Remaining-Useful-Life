package PhysicsBasedPredictionRULv1

block flightCycle
  parameter Real sigma = 2901.8;
  
  parameter Modelica.SIunits.Time period = 300 + 2*taxiRamp + 2*taxiTime + 3*maxRamp + maxTime +cruiseTime;
  parameter Integer nperiod=-1;
  parameter Real offset = 1e-6;
  parameter Modelica.SIunits.Time startTime = 0;
  
  parameter Modelica.SIunits.Time taxiRamp = 600;
  parameter Modelica.SIunits.Time taxiTime = 600;
  parameter Modelica.SIunits.Power taxiP = 0.1*maxP;
  
  parameter Modelica.SIunits.Time maxRamp = 20;
  parameter Modelica.SIunits.Time maxTime = 60;
  parameter Modelica.SIunits.Power maxP = 65000;
  
  parameter Modelica.SIunits.Time cruiseTime = 3600;
  parameter Modelica.SIunits.Power cruiseP = 0.75*maxP;

  Modelica.SIunits.Power totLoad;
  Modelica.SIunits.Power fcLoad;
  //output Modelica.SIunits.Current fcCurrent;
  Modelica.SIunits.Power batLoad;
  output Boolean refuel(start=false);
  //output Modelica.SIunits.Current batCurrent;
  Modelica.Blocks.Noise.NormalNoise normalNoise(mu = cruiseP, samplePeriod = 5, sigma = sigma)  annotation(
    Placement(visible = true, transformation(origin = {-38, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));protected 
  Modelica.SIunits.Time startFlight;
  Integer count;
  
initial algorithm
  count := integer((time-startTime)/period);
  startFlight := startTime + count*period;

equation
  when integer((time - startTime)/period) > pre(count) then
    count = pre(count) + 1;
    startFlight = time;
  end when;

  if mod(time,period) == 0 then
    refuel = true;
  else
    refuel = false;
  end if;
  
  if (time < startTime or nperiod == 0 or (nperiod > 0 and count >= nperiod)) then 
    totLoad = offset;
    fcLoad = offset;
  elseif time < startFlight + taxiRamp then
    totLoad = (taxiP/taxiRamp)*(time - startFlight) + offset;
    fcLoad = totLoad;
  elseif time < startFlight + taxiRamp + taxiTime then
    totLoad = taxiP+offset;
    fcLoad = totLoad;
  elseif time < startFlight  + taxiRamp + taxiTime + maxRamp then
    totLoad = ((maxP - taxiP)/maxRamp)*(time - (startFlight+taxiRamp+taxiTime))+taxiP+offset;
    fcLoad = ((cruiseP -taxiP)/(2*maxRamp + maxTime))*(time-(startFlight+taxiRamp+taxiTime))+taxiP+offset;
  elseif time < startFlight + taxiRamp + taxiTime + maxRamp + maxTime then
    totLoad = maxP+offset;
    fcLoad = ((cruiseP -taxiP)/(2*maxRamp + maxTime))*(time-(startFlight+taxiRamp+taxiTime))+taxiP+offset;
  elseif time < startFlight + taxiRamp + taxiTime + 2*maxRamp + maxTime then
    totLoad = ((cruiseP-maxP)/maxRamp)*(time - (startFlight + taxiRamp + taxiTime + maxRamp + maxTime)) + maxP + offset;
    fcLoad = ((cruiseP -taxiP)/(2*maxRamp + maxTime))*(time-(startFlight+taxiRamp+taxiTime))+taxiP + offset;
  elseif time < startFlight + taxiRamp + taxiTime + 2*maxRamp + maxTime + cruiseTime then
    totLoad = normalNoise.y+offset;
    fcLoad = cruiseP + offset;
  elseif time < startFlight + taxiRamp + taxiTime + 3*maxRamp + maxTime + cruiseTime then
    totLoad = ((taxiP - cruiseP)/maxTime)*(time - (startFlight + taxiRamp + taxiTime + 2*maxRamp + maxTime + cruiseTime)) + cruiseP + offset;
    fcLoad = ((taxiP - cruiseP)/(2*maxRamp+maxTime))*(time - (startFlight+taxiRamp+taxiTime+2*maxRamp+maxTime+cruiseTime)) + cruiseP + offset;
  elseif time < startFlight+taxiRamp+taxiTime + 4*maxRamp + 2*maxTime + cruiseTime then
    totLoad = taxiP + offset;
    fcLoad = ((taxiP - cruiseP)/(2*maxRamp+maxTime))*(time - (startFlight+taxiRamp+taxiTime+2*maxRamp+maxTime+cruiseTime)) + cruiseP + offset;
  elseif time < startFlight + taxiRamp + 2*taxiTime + 3*maxRamp + maxTime + cruiseTime then
    totLoad = taxiP + offset;
    fcLoad = totLoad;
  elseif time< startFlight + 2*taxiRamp + 2*taxiTime + 3*maxRamp + maxTime + cruiseTime then
    totLoad = (-taxiP/taxiRamp)*(time - (startFlight + taxiRamp + 2*taxiTime + 3*maxRamp + maxTime + cruiseTime)) + taxiP + offset;
    fcLoad = totLoad;
  else
    totLoad = offset;
    fcLoad = offset;
  end if;
  
  if time < startFlight + 2*taxiRamp + 2*taxiTime + 3*maxRamp + maxTime + cruiseTime then
    batLoad = totLoad - fcLoad + 2*offset;
  else
    batLoad = -50000;
  end if;
   
annotation(
    uses(Modelica(version = "3.2.3")));
end flightCycle;

model detailedFC
  // Constants
  constant Real R = Modelica.Constants.R;
  constant Real F = Modelica.Constants.F;

  // Exernal Ports
  Modelica.Blocks.Interfaces.RealInput q_air "air flow rate (m^3/s)" annotation(
    Placement(transformation(origin = {-90, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-90, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput p_air "air pressure (Pa)" annotation(
    Placement(transformation(origin = {-60, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-60, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput p_fuel "fuel pressure (Pa)" annotation(
    Placement(transformation(origin = {90, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {90, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput q_fuel "fuel flow rate (m^3/s)"annotation(
    Placement(transformation(origin = {60, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {60, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a hp annotation(
    Placement(transformation(origin = {-8, -100}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-8, -100}, extent = {{-10, -10}, {10, 10}})));

// Parameters
  // May work out equations for some of these
  // See textbook for definitions
  parameter Real N = 65 "Number of cells in series per module";
  parameter Real N_m = 10 "Number of modules";
  parameter Real z = 2 "Electrons transferred in reaction";
  parameter Real x_O2 = 0.21 "Oxygen concentration in air";
  parameter Real x_H2 = 0.99 "Hydrogen concentration in fuel";
  parameter Real x_H2O = 0.01 "Vapor concentration in air";

// Variables
  Real U_O2 "Utilization rate O2";
  Real U_H2 "Utilization rate H2";
  Modelica.SIunits.Pressure p_O2 "Partial pressure O2 in cell";
  Modelica.SIunits.Pressure p_H2 "Partial pressure H2 in cell";
  Modelica.SIunits.Pressure p_H2O "Partial pressure water vapor in cell";
  
  Modelica.Electrical.Analog.Sources.SignalVoltage E_oc annotation(
    Placement(transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Sensors.MultiSensor meter annotation(
    Placement(transformation(origin = {-80, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Electrical.Analog.Interfaces.PositivePin p annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n annotation(
    Placement(transformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Ideal.IdealDiode diode annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage v_conc annotation(
    Placement(visible = true, transformation(origin = {18, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.SignalVoltage v_act annotation(
    Placement(visible = true, transformation(origin = {-14, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.VariableResistor R_ohm annotation(
    Placement(visible = true, transformation(origin = {54, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
equation
  U_O2 = (meter.i*R*hp.T)/(2*z*F*p_air*q_air*x_O2);
  U_H2 = (meter.i*R*hp.T)/(z*F*p_fuel*q_fuel*x_H2);
  p_O2 = p_air*x_O2*(1 - U_O2);
  p_H2 = p_fuel*x_H2*(1 - U_H2);
  p_H2O = p_air*(x_H2O + 2*x_O2*U_O2);
// See Larminie & Dicks, 2003 and Laurencelle et. al. 2001
  if hp.T < 373.15 then
    E_oc.v = N*N_m*((1.228 - ((R*hp.T)/(z*F))*(log(p_H2O/(p_H2*(p_O2^0.5))) + log(101325^0.5))) + ((4.01e-2) - (1.4e-4)*(hp.T - 293.15))*log(0.00824));
  else
    E_oc.v = N*N_m*((1.228 - ((R*hp.T)/(z*F))*(log(1/(p_H2)*(p_O2^0.5)) + log(101325^1.5))) + ((4.01e-2) - (1.4e-4)*(hp.T - 293.15))*log(0.00824));
  end if;
// See Larminie & Dicks, 2003 and Laurencelle et. al., 2001
  if v_act.i <=0 then
    v_act.v = 0;
  elseif ((4.01e-2) - (1.4e-4)*(hp.T - 273.15))*log(v_act.i*(232/1000))>0 then
    v_act.v = N*N_m*(((4.01e-2) - (1.4e-4)*(hp.T - 273.15))*log(v_act.i*(232/1000)));
  else
    v_act.v = 0;
  end if;
// See Larminie & Dicks, 2003 and Laurencelle et. al., 2001
  if hp.T >= 312.15 then
    v_conc.v = N*N_m*(((1.1e-4) - (1.2e-6)*(hp.T - 273.15))*exp((8e-3)*(v_conc.i*(232/1000))));
  else
    v_conc.v = N*N_m*(((3.3e-3) - (8.2e-5)*(hp.T - 273.15))*exp((8e-3)*(v_conc.i*(232/1000))));
  end if;
// See Laurencelle et. al., 2001
  R_ohm.R = N*N_m*(((4.77e-4)-(3.32e-6)*(hp.T-273.15))*(1000/232));
  hp.T = 297.15;
//hp.Q_flow = -meter.power*((1.228*N)/(meter.v) - 1);
  connect(n, E_oc.n) annotation(
    Line(points = {{102, 0}, {90, 0}}, color = {0, 0, 255}));
  connect(meter.pv, p) annotation(
    Line(points = {{-80, 10}, {-94, 10}, {-94, 0}, {-100, 0}}, color = {0, 0, 255}));
  connect(meter.nv, n) annotation(
    Line(points = {{-80, -10}, {-80, -24}, {94, -24}, {94, 0}, {102, 0}}, color = {0, 0, 255}));
  connect(meter.pc, diode.n) annotation(
    Line(points = {{-70, 0}, {-60, 0}}, color = {0, 0, 255}));
  connect(meter.nc, p) annotation(
    Line(points = {{-90, 0}, {-100, 0}}, color = {0, 0, 255}));
  connect(v_act.p, v_conc.n) annotation(
    Line(points = {{-4, 0}, {8, 0}}, color = {0, 0, 255}));
  connect(v_act.n, diode.p) annotation(
    Line(points = {{-24, 0}, {-40, 0}}, color = {0, 0, 255}));
  connect(E_oc.p, R_ohm.p) annotation(
    Line(points = {{70, 0}, {64, 0}}, color = {0, 0, 255}));
  connect(R_ohm.n, v_conc.p) annotation(
    Line(points = {{44, 0}, {28, 0}}, color = {0, 0, 255}));
  annotation(
    uses(Modelica(version = "3.2.3")),
    Diagram);
end detailedFC;

model simpleTank

  Modelica.Blocks.Interfaces.RealOutput p_fuel annotation(
    Placement(transformation(origin = {106, 72}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {106, 72}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput q_fuel annotation(
    Placement(transformation(origin = {106, -38}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {106, -38}, extent = {{-10, -10}, {10, 10}})));

  parameter Modelica.SIunits.Volume V = 7.36238 "Cylinder volume";
  parameter Modelica.SIunits.Pressure P = 16000000 "Cylinder pressure";
  parameter Modelica.SIunits.Temperature T = 293.15 "Cylinder temperature";
  parameter Real Cv = 6 "Regulator flow coefficient";
  parameter Modelica.SIunits.Pressure Preg = 300000 "Regulator outlet pressure";
  
  Real m_fuel;  // mass of gas in tank
  Modelica.Blocks.Continuous.LimIntegrator limIntegrator(k = -1,outMax = (P*V*0.002016)/(8.314*T)*0.8, outMin = 0, use_reset = true)  annotation(
    Placement(visible = true, transformation(origin = {-2, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
algorithm

initial equation
  m_fuel = (P * V * 0.002016) / (8.314 * T);

equation
  limIntegrator.u = -Cv * (94.8/3600) * (P/(1e5)) * 0.017929907 * sqrt((((P-Preg)/P)*0.002016)/(T * 1));
  p_fuel = Preg;
  //der(m_fuel) = -der(limIntegrator.y);
  m_fuel = (P * V * 0.002016) / (8.314 * T)-limIntegrator.y;
  
  q_fuel = (der(limIntegrator.y) * V)/m_fuel;
  
  annotation(
    uses(Modelica(version = "3.2.3")));
end simpleTank;

block LCO_SOC
  extends ElectricalEnergyStorage.Icons.Block;
  parameter Real SOCini "Initial state of charge";
  Modelica.Blocks.Interfaces.RealInput C "Connector of Real input signal" annotation(
    Placement(transformation(extent = {{-100, -70}, {-80, -50}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput SOC "Output signal connector" annotation(
    Placement(transformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput i "Connector of Real input signal" annotation(
    Placement(transformation(extent = {{-100, 50}, {-80, 70}}, rotation = 0)));
  Modelica.Blocks.Math.Division DivisionNormierung annotation(
    Placement(transformation(extent = {{-40, -10}, {-20, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimIntegrator limIntegrator(outMax = 1 - (1e-6), outMin = 1e-6, y_start = SOCini)  annotation(
    Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(i, DivisionNormierung.u1) annotation(
    Line(points = {{-90, 60}, {-60, 60}, {-60, 6}, {-42, 6}}, color = {0, 0, 127}));
  connect(C, DivisionNormierung.u2) annotation(
    Line(points = {{-90, -60}, {-60, -60}, {-60, -6}, {-42, -6}}, color = {0, 0, 127}));
  connect(DivisionNormierung.y, limIntegrator.u) annotation(
    Line(points = {{-18, 0}, {-2, 0}}, color = {0, 0, 127}));
  connect(limIntegrator.y, SOC) annotation(
    Line(points = {{22, 0}, {110, 0}}, color = {0, 0, 127}));
  annotation(
    Diagram(graphics),
    Icon(graphics = {Text(textColor = {0, 0, 130}, extent = {{-80, -40}, {-40, -80}}, textString = "C"), Text(textColor = {0, 0, 130}, extent = {{-80, 80}, {-40, 40}}, textString = "i"), Text(textColor = {0, 0, 130}, extent = {{10, 20}, {90, -20}}, textString = "SOC")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
    uses(Modelica(version = "3.2.3")));
end LCO_SOC;

block LCO_CellCalculator
  import ElectricEnergyStorages = ElectricalEnergyStorage;
  extends ElectricalEnergyStorage.Icons.Block;
  Modelica.Blocks.Interfaces.RealInput i annotation(
    Placement(transformation(extent = {{-100, 10}, {-80, 30}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput SOC annotation(
    Placement(transformation(extent = {{100, 50}, {120, 70}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput SOH annotation(
    Placement(transformation(extent = {{100, -70}, {120, -50}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput C "Connector of Real output signal" annotation(
    Placement(transformation(extent = {{100, -30}, {120, -10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput cycles "Connector of Real output signal" annotation(
    Placement(transformation(extent = {{100, 10}, {120, 30}}, rotation = 0)));
  LCO_Capacity c(capacity = capacity) annotation(
    Placement(transformation(extent = {{20, -10}, {40, -30}}, rotation = 0)));
  ElectricEnergyStorages.Batteries.Components.Calculators.Qabs q(Qini = capacity.aging.Qini) annotation(
    Placement(transformation(extent = {{-20, 0}, {0, 20}}, rotation = 0)));
  ElectricEnergyStorages.Batteries.Components.Calculators.Cycles EquivalentCycles(C0 = capacity.C0, Qini = capacity.aging.Qini) annotation(
    Placement(transformation(extent = {{70, 10}, {90, 30}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput t annotation(
    Placement(transformation(extent = {{-100, -30}, {-80, -10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput T annotation(
    Placement(transformation(extent = {{-100, 50}, {-80, 70}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Z "Connector of Real input signal" annotation(
    Placement(transformation(extent = {{-100, -70}, {-80, -50}})));
  parameter Real SOCini "Initial state of charge";
  parameter Modelica.SIunits.Resistance Z0 "Sum of all initial ohmic impedances Rs0+Rd0[1]+...+Rd0[ns]";
  parameter ElectricalEnergyStorage.CellRecords.Components.ChargeCapacity capacity "Charge capacity";
  parameter ElectricalEnergyStorage.CellRecords.Components.SOH SoH "State of health relevant parameters";
  ElectricalEnergyStorage.Batteries.Components.Calculators.SOHSOS sOH(C0 = capacity.C0, SoH = SoH, Z0 = Z0) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{72, -76}, {92, -56}}, rotation = 0)));
  DOD_mSOC dOD_mSOC(initialSOC = SOCini)  annotation(
    Placement(visible = true, transformation(origin = {30, 46}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  LCO_SOC lco_soc(SOCini = SOCini)  annotation(
    Placement(visible = true, transformation(origin = {80, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(i, q.i) annotation(
    Line(points = {{-90, 20}, {-30, 20}, {-30, 10}, {-19, 10}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(EquivalentCycles.cycles, cycles) annotation(
    Line(points = {{91, 20}, {110, 20}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(q.Qabs, c.Qabs) annotation(
    Line(points = {{1, 10}, {10, 10}, {10, -14}, {21, -14}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(t, c.t) annotation(
    Line(points = {{-90, -20}, {-70, -20}, {-70, -26}, {21, -26}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(i, EquivalentCycles.i) annotation(
    Line(points = {{-90, 20}, {-30, 20}, {-30, 26}, {71, 26}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(T, c.T) annotation(
    Line(points = {{-90, 60}, {-48, 60}, {-48, -20}, {21, -20}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(c.C, C) annotation(
    Line(points = {{41, -20}, {110, -20}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(c.C, EquivalentCycles.C) annotation(
    Line(points = {{41, -20}, {60, -20}, {60, 14}, {71, 14}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(c.C, sOH.C) annotation(
    Line(points = {{41, -20}, {60, -20}, {60, -60}, {73, -60}}, color = {0, 0, 127}));
  connect(sOH.Z, Z) annotation(
    Line(points = {{73, -72}, {50, -72}, {50, -60}, {-90, -60}}, color = {0, 0, 127}));
  connect(sOH.SOH, SOH) annotation(
    Line(points = {{93, -60}, {110, -60}}, color = {0, 0, 127}));
  connect(EquivalentCycles.cycles, c.ECF) annotation(
    Line(points = {{92, 20}, {96, 20}, {96, 2}, {30, 2}, {30, -10}}, color = {0, 0, 127}));
  connect(dOD_mSOC.mSOC, c.mSOC) annotation(
    Line(points = {{28, 36}, {26, 36}, {26, -10}}, color = {0, 0, 127}));
  connect(dOD_mSOC.DOD, c.DOD) annotation(
    Line(points = {{34, 36}, {34, -10}}, color = {0, 0, 127}));
  connect(lco_soc.SOC, SOC) annotation(
    Line(points = {{92, 60}, {110, 60}}, color = {0, 0, 127}));
  connect(lco_soc.SOC, dOD_mSOC.SOC) annotation(
    Line(points = {{92, 60}, {96, 60}, {96, 46}, {52, 46}, {52, 62}, {30, 62}, {30, 56}}, color = {0, 0, 127}));
  connect(i, lco_soc.i) annotation(
    Line(points = {{-90, 20}, {-30, 20}, {-30, 66}, {72, 66}}, color = {0, 0, 127}));
  connect(c.C, lco_soc.C) annotation(
    Line(points = {{42, -20}, {60, -20}, {60, 54}, {72, 54}}, color = {0, 0, 127}));
  annotation(
    Diagram(graphics),
    Icon(graphics = {Text(textColor = {0, 0, 130}, extent = {{-86, 44}, {-46, 4}}, textString = "i"), Text(textColor = {0, 0, 130}, extent = {{38, 0}, {90, -40}}, textString = "C"), Text(textColor = {0, 0, 130}, extent = {{6, 80}, {86, 40}}, textString = "SOC"), Text(textColor = {0, 0, 130}, extent = {{-12, 44}, {90, 2}}, textString = "cycles"), Text(textColor = {0, 0, 130}, extent = {{0, -38}, {96, -78}}, textString = "SOH"), Text(textColor = {0, 0, 130}, extent = {{-86, 82}, {-46, 42}}, textString = "T"), Text(textColor = {0, 0, 130}, extent = {{-86, 6}, {-46, -34}}, textString = "t"), Text(textColor = {0, 0, 130}, extent = {{-84, -40}, {-44, -80}}, textString = "Z")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
    uses(ElectricalEnergyStorage(version = "3.2.2"), Modelica(version = "3.2.3")));
end LCO_CellCalculator;

block LCO_Capacity
  extends ElectricalEnergyStorage.Icons.Block;
  parameter ElectricalEnergyStorage.CellRecords.Components.ChargeCapacity capacity "Charge capacity";
  parameter Real b = 0.453;
  Real CF "Capacity fade (%)";
  Real RUL "Remaining Useful Life (%)";
  Modelica.Blocks.Interfaces.RealInput ECF annotation(
    Placement(visible = true, transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(extent = {{-100, -10}, {-80, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput DOD annotation(
    Placement(visible = true, transformation(origin = {80, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {40, 0}, extent = {{-100, -10}, {-80, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput mSOC annotation(
    Placement(visible = true, transformation(origin = {-80, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {-40, 0}, extent = {{-100, -10}, {-80, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput Qabs "Connector of Real input signal" annotation(
    Placement(transformation(extent = {{-100, -70}, {-80, -50}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput T "Connector of Real input signal" annotation(
    Placement(transformation(extent = {{-100, -10}, {-80, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput t "Connector of Real input signal" annotation(
    Placement(transformation(extent = {{-100, 50}, {-80, 70}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput C "Output signal connector" annotation(
    Placement(transformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
  
equation
  //C = capacity.C0*(1 + capacity.aging.kt*t + capacity.aging.kQabs*Qabs)*(1 + capacity.temperature.alpha*(T - capacity.temperature.Tref));
  CF = 1-(C/capacity.C0);
  RUL = 1-(CF/0.2);
  C = (capacity.C0/100)*(100-(3.25*mSOC*(1+3.25*DOD-2.25*(DOD^2)))*((ECF)^b));
  annotation(
    Diagram(graphics),
    Icon(graphics = {Text(textColor = {0, 0, 130}, extent = {{-108, 82}, {-28, 42}}, textString = "t"), Text(textColor = {0, 0, 130}, extent = {{40, 20}, {100, -20}}, textString = "C"), Text(textColor = {0, 0, 130}, extent = {{-106, -36}, {-4, -76}}, textString = "|Q|"), Text(textColor = {0, 0, 130}, extent = {{-106, 20}, {-26, -20}}, textString = "T")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
    uses(Modelica(version = "3.2.3")));
end LCO_Capacity;

block LCO_StaticResistanceScaled
import ElectricEnergyStorages = ElectricalEnergyStorage;
extends ElectricalEnergyStorage.Icons.StackStaticResistance;
  parameter Integer ns(min = 1) "number of serial connected cells";
  parameter Integer np(min = 1) "number of parallel connected cells";
  parameter ElectricalEnergyStorage.CellRecords.StaticResistance.StaticResistanceParameters cellParameters annotation(
    __Dymola_choicesAllMatching = true,
    Placement(transformation(extent = {{-100, 80}, {-80, 100}})));
  extends ElectricalEnergyStorage.Batteries.Components.OperationalParameters;
  parameter Real SOCini(start = 0.5) "Initial state of charge" annotation(
    Dialog(group = "Initialization"));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n "Negative pin" annotation(
    Placement(transformation(extent = {{-10, -110}, {10, -90}}, rotation = 0), iconTransformation(extent = {{-10, -104}, {10, -84}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p "Positive pin (potential p.v > n.v for positive voltage drop v)" annotation(
    Placement(transformation(extent = {{-10, 90}, {10, 110}}, rotation = 0), iconTransformation(extent = {{-10, 84}, {10, 104}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage Uo annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-10, 10}, {10, -10}}, rotation = 270)));
  Modelica.Electrical.Analog.Sensors.CurrentSensor IBatt annotation(
    Placement(transformation(origin = {0, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 270)));
  Modelica.Electrical.Analog.Basic.HeatingResistor Rs(useHeatPort = true, final R_ref = cellParameters.Rs.R0*ns/np, final T_ref = cellParameters.Rs.temperature.Tref, final alpha = cellParameters.Rs.temperature.alpha) annotation(
    Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort if useHeatPort annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}}, rotation = 0), iconTransformation(extent = {{84, -10}, {104, 10}})));
  Modelica.Blocks.Math.Gain gain(final k = ns) annotation(
    Placement(transformation(extent = {{-40, -80}, {-20, -60}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(final k = 1/np) annotation(
    Placement(transformation(origin = {-30, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
protected
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a internalHeatPort annotation(
    Placement(transformation(extent = {{76, -4}, {84, 4}})));
public
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(final T = TOperational) if not useHeatPort annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {80, -30})));
  Modelica.Blocks.Sources.Clock clock(final startTime = tini) annotation(
    Placement(transformation(extent = {{-4, 4}, {4, -4}}, rotation = 180, origin = {-24, 36})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temp annotation(
    Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 0, origin = {70, 80})));
  Modelica.Blocks.Sources.Constant const(final k = cellParameters.Rs.R0) annotation(
    Placement(transformation(extent = {{4, -4}, {-4, 4}}, rotation = 0, origin = {-24, 24})));
  LCO_CellCalculator cellCalculator(final SOCini = SOCini, final Z0 = cellParameters.Rs.R0, final capacity = cellParameters.capacity, final SoH = cellParameters.SoH) annotation(
    Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-70, 58})));
  Modelica.Blocks.Tables.CombiTable1D sococvTable(final smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative, final tableOnFile = cellParameters.SOCOCV.OCVtableOnFile, final table = cellParameters.SOCOCV.OCVtable, final tableName = cellParameters.SOCOCV.OCVtableName, final fileName = cellParameters.SOCOCV.OCVfileName) annotation(
    Placement(transformation(extent = {{-70, -80}, {-50, -60}}, rotation = 0)));
  final parameter Modelica.SIunits.Time tini(fixed = false) "Initial time";
initial equation
  tini = time;
equation
  connect(Uo.n, pin_n) annotation(
    Line(points = {{0, -80}, {0, -100}}, color = {0, 0, 255}));
  connect(Rs.p, Uo.p) annotation(
    Line(points = {{0, -10}, {0, -35}, {0, -60}}, color = {0, 0, 255}));
  connect(gain.y, Uo.v) annotation(
    Line(points = {{-19, -70}, {-7, -70}}, color = {0, 0, 127}));
  connect(gain1.u, IBatt.i) annotation(
    Line(points = {{-18, 60}, {-10, 60}}, color = {0, 0, 127}));
  connect(Rs.heatPort, internalHeatPort) annotation(
    Line(points = {{10, 0}, {80, 0}}, color = {191, 0, 0}, smooth = Smooth.None));
  connect(internalHeatPort, fixedTemperature.port) annotation(
    Line(points = {{80, 0}, {80, -20}}, color = {191, 0, 0}, smooth = Smooth.None));
  connect(internalHeatPort, heatPort) annotation(
    Line(points = {{80, 0}, {100, 0}}, color = {191, 0, 0}, smooth = Smooth.None));
  connect(internalHeatPort, temp.port) annotation(
    Line(points = {{80, 0}, {80, 80}}, color = {191, 0, 0}, smooth = Smooth.None));
  connect(cellCalculator.i, gain1.y) annotation(
    Line(points = {{-61, 60}, {-41, 60}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(cellCalculator.T, temp.T) annotation(
    Line(points = {{-61, 64}, {-40, 64}, {-40, 80}, {60, 80}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(cellCalculator.t, clock.y) annotation(
    Line(points = {{-61, 56}, {-40, 56}, {-40, 36}, {-28.4, 36}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(const.y, cellCalculator.Z) annotation(
    Line(points = {{-28.4, 24}, {-50, 24}, {-50, 52}, {-61, 52}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(IBatt.p, Rs.n) annotation(
    Line(points = {{0, 50}, {0, 10}}, color = {0, 0, 255}, smooth = Smooth.None));
  connect(IBatt.n, pin_p) annotation(
    Line(points = {{0, 70}, {0, 100}}, color = {0, 0, 255}, smooth = Smooth.None));
  connect(sococvTable.y[1], gain.u) annotation(
    Line(points = {{-49, -70}, {-42, -70}}, color = {0, 0, 127}, smooth = Smooth.None));
  connect(sococvTable.u[1], cellCalculator.SOC) annotation(
    Line(points = {{-72, -70}, {-90, -70}, {-90, 64}, {-81, 64}}, color = {0, 0, 127}, smooth = Smooth.None));
  annotation(
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1), graphics),
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1), graphics));

end LCO_StaticResistanceScaled;

block DOD_mSOC
  extends ElectricalEnergyStorage.Icons.Block;
  Modelica.Blocks.Interfaces.RealInput SOC annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-100, -10}, {-80, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput DOD annotation(
    Placement(visible = true, transformation(origin = {100, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput mSOC annotation(
    Placement(visible = true, transformation(origin = {100, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Real initialSOC;
  Real maxSOC(start=initialSOC);
  Real minSOC(start=initialSOC);
  
initial equation
  //maxSOC = initialSOC;
  //minSOC = initialSOC;

algorithm
  maxSOC := max(delay(maxSOC,1),SOC);
  minSOC := min(delay(minSOC,1),SOC);
  
equation
  DOD = maxSOC - minSOC;
  mSOC = (DOD/2)+minSOC;
  annotation(
    uses(Modelica(version = "3.2.3")));
end DOD_mSOC;

model LCO_CellStackTC
  extends LCO_StaticResistanceScaled;
  parameter Real RUL_Start = 1;
  parameter Real CycleLife = 10;

equation

end LCO_CellStackTC;

end PhysicsBasedPredictionRULv1;