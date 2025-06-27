package PhysicsBasedPredictionRULv101
  block flightCycle
    parameter Real sigma = 2901.8;
    parameter Modelica.Units.SI.Time period = 300 + 2*taxiRamp + 2*taxiTime + 3*maxRamp + maxTime + cruiseTime;
    parameter Integer nperiod = -1;
    parameter Real offset = 1e-6;
    parameter Modelica.Units.SI.Time startTime = 0;
    parameter Modelica.Units.SI.Time taxiRamp = 600;
    parameter Modelica.Units.SI.Time taxiTime = 600;
    parameter Modelica.Units.SI.Power taxiP = 0.1*maxP;
    parameter Modelica.Units.SI.Time maxRamp = 20;
    parameter Modelica.Units.SI.Time maxTime = 60;
    parameter Modelica.Units.SI.Power maxP = 65000;
    parameter Modelica.Units.SI.Time cruiseTime = 3600;
    parameter Modelica.Units.SI.Power cruiseP = 0.75*maxP;
    Modelica.Units.SI.Power totLoad;
    Modelica.Units.SI.Power fcLoad;
    //output Modelica.SIunits.Current fcCurrent;
    Modelica.Units.SI.Power batLoad;
    output Boolean refuel(start = false);
    //output Modelica.SIunits.Current batCurrent;
    Modelica.Blocks.Noise.NormalNoise normalNoise(mu = cruiseP, samplePeriod = 5, sigma = sigma) annotation(
      Placement(visible = true, transformation(origin = {-38, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    Modelica.Units.SI.Time startFlight;
    Integer count;
  initial algorithm
    count := integer((time - startTime)/period);
    startFlight := startTime + count*period;
  equation
    when integer((time - startTime)/period) > pre(count) then
      count = pre(count) + 1;
      startFlight = time;
    end when;
    if mod(time, period) == 0 then
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
      totLoad = taxiP + offset;
      fcLoad = totLoad;
    elseif time < startFlight + taxiRamp + taxiTime + maxRamp then
      totLoad = ((maxP - taxiP)/maxRamp)*(time - (startFlight + taxiRamp + taxiTime)) + taxiP + offset;
      fcLoad = ((cruiseP - taxiP)/(2*maxRamp + maxTime))*(time - (startFlight + taxiRamp + taxiTime)) + taxiP + offset;
    elseif time < startFlight + taxiRamp + taxiTime + maxRamp + maxTime then
      totLoad = maxP + offset;
      fcLoad = ((cruiseP - taxiP)/(2*maxRamp + maxTime))*(time - (startFlight + taxiRamp + taxiTime)) + taxiP + offset;
    elseif time < startFlight + taxiRamp + taxiTime + 2*maxRamp + maxTime then
      totLoad = ((cruiseP - maxP)/maxRamp)*(time - (startFlight + taxiRamp + taxiTime + maxRamp + maxTime)) + maxP + offset;
      fcLoad = ((cruiseP - taxiP)/(2*maxRamp + maxTime))*(time - (startFlight + taxiRamp + taxiTime)) + taxiP + offset;
    elseif time < startFlight + taxiRamp + taxiTime + 2*maxRamp + maxTime + cruiseTime then
      totLoad = normalNoise.y + offset;
      fcLoad = cruiseP + offset;
    elseif time < startFlight + taxiRamp + taxiTime + 3*maxRamp + maxTime + cruiseTime then
      totLoad = ((taxiP - cruiseP)/maxTime)*(time - (startFlight + taxiRamp + taxiTime + 2*maxRamp + maxTime + cruiseTime)) + cruiseP + offset;
      fcLoad = ((taxiP - cruiseP)/(2*maxRamp + maxTime))*(time - (startFlight + taxiRamp + taxiTime + 2*maxRamp + maxTime + cruiseTime)) + cruiseP + offset;
    elseif time < startFlight + taxiRamp + taxiTime + 4*maxRamp + 2*maxTime + cruiseTime then
      totLoad = taxiP + offset;
      fcLoad = ((taxiP - cruiseP)/(2*maxRamp + maxTime))*(time - (startFlight + taxiRamp + taxiTime + 2*maxRamp + maxTime + cruiseTime)) + cruiseP + offset;
    elseif time < startFlight + taxiRamp + 2*taxiTime + 3*maxRamp + maxTime + cruiseTime then
      totLoad = taxiP + offset;
      fcLoad = totLoad;
    elseif time < startFlight + 2*taxiRamp + 2*taxiTime + 3*maxRamp + maxTime + cruiseTime then
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
    Modelica.Blocks.Interfaces.RealInput q_fuel "fuel flow rate (m^3/s)" annotation(
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
    Modelica.Units.SI.Pressure p_O2 "Partial pressure O2 in cell";
    Modelica.Units.SI.Pressure p_H2 "Partial pressure H2 in cell";
    Modelica.Units.SI.Pressure p_H2O "Partial pressure water vapor in cell";
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
    if v_act.i <= 0 then
      v_act.v = 0;
    elseif ((4.01e-2) - (1.4e-4)*(hp.T - 273.15))*log(v_act.i*(232/1000)) > 0 then
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
    R_ohm.R = N*N_m*(((4.77e-4) - (3.32e-6)*(hp.T - 273.15))*(1000/232));
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
    parameter Modelica.Units.SI.Volume V = 7.36238 "Cylinder volume";
    parameter Modelica.Units.SI.Pressure P = 16000000 "Cylinder pressure";
    parameter Modelica.Units.SI.Temperature T = 293.15 "Cylinder temperature";
    parameter Real Cv = 6 "Regulator flow coefficient";
    parameter Modelica.Units.SI.Pressure Preg = 300000 "Regulator outlet pressure";
    Real m_fuel;
    // mass of gas in tank
    Modelica.Blocks.Continuous.LimIntegrator limIntegrator(k = -1, outMax = (P*V*0.002016)/(8.314*T)*0.8, outMin = 0, use_reset = true) annotation(
      Placement(visible = true, transformation(origin = {-2, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  algorithm

  initial equation
    m_fuel = (P*V*0.002016)/(8.314*T);
  equation
    limIntegrator.u = -Cv*(94.8/3600)*(P/(1e5))*0.017929907*sqrt((((P - Preg)/P)*0.002016)/(T*1));
    p_fuel = Preg;
//der(m_fuel) = -der(limIntegrator.y);
    m_fuel = (P*V*0.002016)/(8.314*T) - limIntegrator.y;
    q_fuel = (der(limIntegrator.y)*V)/m_fuel;
    annotation(
      uses(Modelica(version = "3.2.3")));
  end simpleTank;

  record CellParameters
    extends Modelica.Electrical.Batteries.Icons.BaseCellRecord;
    parameter Modelica.Units.SI.ElectricCharge Qnom(displayUnit = "A.h") "Nominal (maximum) charge";
    parameter Modelica.Units.SI.Resistance Ri "Total inner resistance (= OCVmax/Isc)";
    parameter Modelica.Units.SI.Temperature T_ref = 293.15 "Reference temperature";
    parameter Modelica.Units.SI.LinearTemperatureCoefficient alpha = 0 "Temperature coefficient of resistance at T_ref";
    parameter Modelica.Units.SI.Current Idis = 0 "Self-discharge current at SOC = SOCmax" annotation(
      Evaluate = true);
    parameter Modelica.Units.SI.Resistance R0 = Ri "Inner resistance without parallel C";
    parameter Boolean OCVtableOnFile = false "true, if OCV table is defined on file or in function usertab";
    parameter Real OCVtable[:, 2] = [0, 2.7; 0.0085, 3.313; 0.05, 3.35; 0.1, 3.49; 0.2, 3.55; 0.4, 3.65; 0.6, 3.75; 0.75, 3.85; 0.9, 4; 1, 4.2] "SOC = first column, OCV = second column";
    parameter String OCVtableName = "NoName" "OCV table name on file or in function usertab";
    parameter String OCVfileName = "NoName" "file where OCV matrix is stored";
  end CellParameters;

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
    Real maxSOC(start = initSOC);
    Real minSOC(start = initSOC);
    Modelica.Blocks.Math.Abs abs1 annotation(
      Placement(transformation(origin = {-20, -20}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Gain gain1(k = 0.5) annotation(
      Placement(transformation(origin = {10, -20}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Division division1 annotation(
      Placement(transformation(origin = {46, -40}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Continuous.Integrator integrator1 annotation(
      Placement(transformation(origin = {76, -40}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Division division2 annotation(
      Placement(transformation(origin = {40, 78}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Continuous.LimIntegrator integrator2(outMax = 1 - (1e-6), outMin = 1e-6, y_start = initSOC) annotation(
      Placement(transformation(origin = {74, 78}, extent = {{-10, -10}, {10, 10}})));
  algorithm
    maxSOC := max(delay(maxSOC, 1), SOC);
    minSOC := min(delay(minSOC, 1), SOC);
  equation
    CF = 1 - (capacity/Q0);
    RUL = 1 - (CF/0.2);
    capacity = (Q0/100)*(100 - (3.25*mSOC*(1 + 3.25*DoD - 2.25*(DoD^2)))*((cycles)^b));
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

  model LCO_Stack "Battery with open-circuit voltage dependent on state of charge, self-discharge and inner resistance"
    extends Modelica.Electrical.Batteries.Icons.BatteryIcon(final displaySOC = cellCalculator.SOC);
    parameter Integer Ns(final min = 1) = 1 "Number of serial connected cells";
    parameter Integer Np(final min = 1) = 1 "Number of parallel connected cells";
    parameter Real SOCtolerance = 1e-9 "Tolerance to detect depleted of overcharged battery" annotation(
      Dialog(tab = "Advanced"));
    parameter Real initSOC;
    extends Modelica.Electrical.Analog.Interfaces.TwoPin;
    Modelica.Units.SI.Current i = p.i "Current into the battery";
    Modelica.Units.SI.Power power = v*i "Power to the battery" annotation(
      Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
    //output Real SOC(start = 1) = cellCalculator.SOC "State of charge" annotation(
    //Dialog(showStartAttribute = true));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor annotation(
      Placement(transformation(extent = {{-90, 10}, {-70, -10}})));
    Modelica.Blocks.Math.Gain gainV(final k = Ns) annotation(
      Placement(transformation(origin = {-40, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Analog.Sources.SignalVoltage ocv annotation(
      Placement(transformation(extent = {{-50, -10}, {-30, 10}})));
  Modelica.Electrical.Analog.Basic.Conductor selfDischarge(final G = Np*cellData.Idis/(Ns), T_ref = 293.15, final useHeatPort = true) annotation(
      Placement(transformation(origin = {8, 0}, extent = {{-70, -30}, {-50, -10}})));
    Modelica.Electrical.Analog.Basic.Resistor r0(final R = cellData.Ri*Ns/Np, final T_ref = cellData.T_ref, final alpha = cellData.alpha, final useHeatPort = true) annotation(
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    extends Modelica.Electrical.Analog.Interfaces.PartialConditionalHeatPort;
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temp annotation(
      Placement(transformation(origin = {-20, -80}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Blocks.Tables.CombiTable1Dv ocv_soc(final tableOnFile = cellData.OCVtableOnFile, final table = cellData.OCVtable, final tableName = cellData.OCVtableName, final fileName = cellData.OCVfileName, final smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(transformation(origin = {-60, 86}, extent = {{-10, -10}, {10, 10}})));
    CellParameters cellData annotation(
      Placement(transformation(origin = {24, 72}, extent = {{-10, -10}, {10, 10}})));
    LCO_cellCalculator cellCalculator(initSOC = initSOC, Q0 = cellData.Qnom) annotation(
      Placement(transformation(origin = {-74, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain1(k = -1/Np)  annotation(
      Placement(transformation(origin = {-80, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  initial equation
//cellCalculator.SOC = initSOC;
  equation
    connect(gainV.y, ocv.v) annotation(
      Line(points = {{-40, 47}, {-40, 12}}, color = {0, 0, 127}));
    connect(currentSensor.n, ocv.p) annotation(
      Line(points = {{-70, 0}, {-50, 0}}, color = {0, 0, 255}));
    connect(p, currentSensor.p) annotation(
      Line(points = {{-100, 0}, {-90, 0}}, color = {0, 0, 255}));
    connect(currentSensor.p, selfDischarge.p) annotation(
      Line(points = {{-90, 0}, {-90, -20}, {-62, -20}}, color = {0, 0, 255}));
    connect(ocv.n, selfDischarge.n) annotation(
      Line(points = {{-30, 0}, {-30, -20}, {-42, -20}}, color = {0, 0, 255}));
    connect(selfDischarge.heatPort, internalHeatPort) annotation(
      Line(points = {{-52, -30}, {-52, -40}, {0, -40}, {0, -80}}, color = {191, 0, 0}));
    connect(internalHeatPort, r0.heatPort) annotation(
      Line(points = {{0, -80}, {0, -10}}, color = {191, 0, 0}));
    connect(internalHeatPort, temp.port) annotation(
      Line(points = {{0, -80}, {-10, -80}}, color = {191, 0, 0}));
    connect(ocv_soc.y[1], gainV.u) annotation(
      Line(points = {{-49, 86}, {-41, 86}, {-41, 70}}, color = {0, 0, 127}));
    connect(temp.T, cellCalculator.T) annotation(
      Line(points = {{-30, -80}, {-30, -79}, {-70, -79}, {-70, 47}}, color = {0, 0, 127}));
    connect(cellCalculator.SOC, ocv_soc.u[1]) annotation(
      Line(points = {{-81.8, 68.6}, {-81.8, 86.6}, {-71.8, 86.6}}, color = {0, 0, 127}));
    connect(currentSensor.i, gain1.u) annotation(
      Line(points = {{-80, 12}, {-80, 16}}, color = {0, 0, 127}));
    connect(gain1.y, cellCalculator.i) annotation(
      Line(points = {{-80, 40}, {-80, 48}}, color = {0, 0, 127}));
  connect(r0.p, n) annotation(
      Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 255}));
  connect(r0.n, ocv.n) annotation(
      Line(points = {{-10, 0}, {-30, 0}}, color = {0, 0, 255}));
    annotation(
      uses(Modelica(version = "4.0.0")),
      Diagram);
  end LCO_Stack;
  annotation(
    uses(Modelica(version = "4.0.0")));
end PhysicsBasedPredictionRULv101;
