within Buildings.Experimental.DHC.Loads.Validation;
model EnergyMassFlow
  extends Modelica.Icons.Example;
  package Medium = Buildings.Media.Water
    "Source side medium";
  constant Modelica.SIunits.SpecificHeatCapacity cpWatLiq=
    Buildings.Utilities.Psychrometrics.Constants.cpWatLiq
    "Specific heat capacity of liquid water";
  parameter String filNam=Modelica.Utilities.Files.loadResource(
    "modelica://Buildings/Resources/Data/Experimental/DHC/Loads/Examples/SwissResidential_20190916.mos")
    "Path of the file with loads as time series";
  final parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal=
    Buildings.Experimental.DHC.Loads.BaseClasses.getPeakLoad(
      string="#Peak space heating load",
      filNam=Modelica.Utilities.Files.loadResource(filNam))
    "Design heating heat flow rate (>=0)"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.PressureDifference dp_nominal=30000
    "Nominal pressure drop";
  parameter Real fra_m_flow_min = 0.1
    "Minimum flow rate (ratio to nominal)";
  parameter Modelica.SIunits.Temperature TSupSet_nominal=333.15
    "Supply temperature set point at nominal conditions";
  parameter Modelica.SIunits.Temperature TSupSer_nominal=TSupSet_nominal + 10
    "Service supply temperature at nominal conditions";
  parameter Boolean have_reset = false
    "Set to true to reset the supply temperature (consider enumeration for open loop reset based on TOut or closed loop based on load signal (approximating T&R)";
  parameter Modelica.SIunits.Time tau = 60
    "Time constant at nominal flow"
    annotation (Dialog(tab="Dynamics", group="Nominal condition"));
  Modelica.Blocks.Sources.CombiTimeTable loa(
    tableOnFile=true,
    tableName="tab1",
    fileName=filNam,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    y(each unit="W"),
    offset={0,0,0},
    columns={2,3,4},
    smoothness=Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative1)
    "Reader for thermal loads (y[1] is cooling load, y[2] is heating load)"
    annotation (Placement(transformation(extent={{-210,110},{-190,130}})));
  Fluid.HeatExchangers.PlateHeatExchangerEffectivenessNTU hex(
    redeclare package Medium1=Medium,
    redeclare package Medium2=Medium,
    m1_flow_nominal=m_flow_nominal,
    m2_flow_nominal=m_flow_nominal,
    dp1_nominal=0,
    dp2_nominal=dp_nominal,
    configuration=Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow,
    Q_flow_nominal=Q_flow_nominal,
    T_a1_nominal=TSupSer_nominal,
    T_a2_nominal=TSupSet_nominal - dT1_nominal) "ETS HX"
   annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-114,-40})));
  Fluid.Sensors.TemperatureTwoPort senTSup(redeclare package Medium = Medium,
      m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-90,-30},{-70,-10}})));
  Fluid.Sources.Boundary_pT serSup(
    redeclare package Medium = Medium,
    p=Medium.p_default + 10E4,
    T=TSupSer_nominal - 40,
    nPorts=1) "Service supply"
    annotation (Placement(transformation(extent={{-210,-30},{-190,-10}})));
  Fluid.Actuators.Valves.TwoWayEqualPercentage val(
    redeclare package Medium=Medium,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=dp_nominal,
    dpFixed_nominal=dp_nominal)
    annotation (Placement(transformation(extent={{-150,-30},{-130,-10}})));
  Fluid.Sources.Boundary_pT serRet(redeclare package Medium = Medium, nPorts=1)
    "Service return"
    annotation (Placement(transformation(extent={{-210,-70},{-190,-50}})));
  Buildings.Controls.OBC.CDL.Continuous.PID conPID(k=0.01, Ti=60)
    annotation (Placement(transformation(extent={{-170,10},{-150,30}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant temSupSet(
    y(final unit="K", displayUnit="degC"),
    k=TSupSet_nominal)
    annotation (Placement(transformation(extent={{-210,50},{-190,70}})));
  Fluid.Movers.FlowControlled_m_flow pum(
    redeclare package Medium=Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=m_flow_nominal,
    per(pressure(V_flow=m_flow_nominal/1000*{0,1,2}, dp=dp_nominal*{2,1,0})),
    use_inputFilter=false,
    dp_nominal=dp_nominal)
    annotation (Placement(transformation(extent={{30,-30},{50,-10}})));
  .Buildings.Experimental.DHC.Loads.EnergyMassFlow masFlo(
    have_pum=true,
    tau=tau,
    Q_flow_nominal=Q_flow_nominal,
    m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{0,30},{20,50}})));
  Fluid.Delays.DelayFirstOrder del(
    redeclare package Medium=Medium,
    tau=10*tau,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    m_flow_nominal=m_flow_nominal, nPorts=3)
    annotation (Placement(transformation(extent={{90,-20},{110,0}})));
  HeatTransfer.Sources.PrescribedHeatFlow heaFlo "Heat flow rate" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={80,10})));
  Fluid.Sensors.TemperatureTwoPort senTRet(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-70,-70},{-90,-50}})));
  Fluid.Sources.Boundary_pT bou2(
    redeclare package Medium=Medium, nPorts=1)
              annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={100,-40})));
  Buildings.Controls.OBC.CDL.Continuous.MovingMean movMea(delta=10*tau)
    annotation (Placement(transformation(extent={{110,130},{130,150}})));
  Buildings.Controls.OBC.CDL.Continuous.Add add2(k2=-1)
    annotation (Placement(transformation(extent={{60,110},{80,130}})));
  Fluid.HeatExchangers.HeaterCooler_u hea(
    redeclare package Medium=Medium,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=dp_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    Q_flow_nominal=1)
    annotation (Placement(transformation(extent={{130,-30},{150,-10}})));
  Utilities.Math.IntegratorWithReset int(reset=Buildings.Types.Reset.Parameter)
    annotation (Placement(transformation(extent={{110,90},{130,110}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=time > 1E4)
    "Reset to cancel the effect of the warmup period"
    annotation (Placement(transformation(extent={{80,50},{100,70}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=hex.Q2_flow)
    annotation (Placement(transformation(extent={{10,90},{30,110}})));
  Utilities.Math.IntegratorWithReset int1(
    y_start=1E-6,
    reset=Buildings.Types.Reset.Parameter,
    y_reset=1E-6)
    annotation (Placement(transformation(extent={{150,70},{170,90}})));
  Buildings.Controls.OBC.CDL.Continuous.Division NMBE
    annotation (Placement(transformation(extent={{190,80},{210,100}})));
  Fluid.Sensors.MassFlowRate senMasFlo(
    redeclare package Medium=Medium)
    annotation (Placement(transformation(extent={{-50,-30},{-30,-10}})));
  Fluid.Sensors.TemperatureTwoPort senTSup1(redeclare package Medium = Medium,
      m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-90,-190},{-70,-170}})));
  Fluid.Sources.Boundary_pT serSup1(
    redeclare package Medium = Medium,
    p=Medium.p_default + 10E4,
    T=TSupSer_nominal - 40,
    nPorts=1) "Service supply"
    annotation (Placement(transformation(extent={{-210,-190},{-190,-170}})));
  Fluid.Sources.Boundary_pT serRet1(redeclare package Medium = Medium, nPorts=1)
    "Service return"
    annotation (Placement(transformation(extent={{-210,-230},{-190,-210}})));
  Fluid.Actuators.Valves.TwoWayPressureIndependent valPreInd(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=dp_nominal,
    use_inputFilter=false)
    annotation (Placement(transformation(extent={{30,-190},{50,-170}})));
  Buildings.Experimental.DHC.Loads.EnergyMassFlow  masFlo1(
    have_pum=false,
    tau=tau,
    Q_flow_nominal=Q_flow_nominal,
    m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{0,-130},{20,-110}})));
  Fluid.Delays.DelayFirstOrder del1(
    redeclare package Medium = Medium,
    tau=10*tau,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    m_flow_nominal=m_flow_nominal,
    nPorts=3)
    annotation (Placement(transformation(extent={{90,-180},{110,-160}})));
  HeatTransfer.Sources.PrescribedHeatFlow heaFlo1
                                                 "Heat flow rate" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={80,-150})));
  Fluid.Sensors.TemperatureTwoPort senTRet1(redeclare package Medium = Medium,
      m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-70,-230},{-90,-210}})));
  Fluid.Sources.Boundary_pT bou1(
    redeclare package Medium = Medium, nPorts=1)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={100,-200})));
  Fluid.HeatExchangers.HeaterCooler_u hea1(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=dp_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    Q_flow_nominal=1)
    annotation (Placement(transformation(extent={{130,-190},{150,-170}})));
  Fluid.Sensors.MassFlowRate senMasFlo1(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-50,-190},{-30,-170}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai(k=1/m_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={40,-150})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant FIXME(
    k=0)
    annotation (Placement(transformation(extent={{80,-110},{100,-90}})));
protected
  parameter Modelica.SIunits.TemperatureDifference dT1_nominal = 20
    "Nominal Delta-T: change default btw cooling and heating applications";
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal=
    abs(Q_flow_nominal / dT1_nominal / cpWatLiq)
    "Nominal mass flow rate";
equation
  connect(hex.port_b2, senTSup.port_a)
    annotation (Line(points={{-108,-30},{-108,-20},{-90,-20}},
                                                          color={0,127,255}));
  connect(val.port_b, hex.port_a1)
    annotation (Line(points={{-130,-20},{-120,-20},{-120,-30}},
                                                          color={0,127,255}));
  connect(serSup.ports[1], val.port_a)
    annotation (Line(points={{-190,-20},{-150,-20}},
                                                   color={0,127,255}));
  connect(serRet.ports[1], hex.port_b1) annotation (Line(points={{-190,-60},{-120,
          -60},{-120,-50}}, color={0,127,255}));
  connect(senTSup.T, conPID.u_m) annotation (Line(points={{-80,-9},{-80,0},{-160,
          0},{-160,8}},   color={0,0,127}));
  connect(conPID.y, val.y)
    annotation (Line(points={{-148,20},{-140,20},{-140,-8}},
                                                          color={0,0,127}));
  connect(temSupSet.y, conPID.u_s)
    annotation (Line(points={{-188,60},{-180,60},{-180,20},{-172,20}},
                                                   color={0,0,127}));
  connect(loa.y[2], masFlo.Q_flow) annotation (Line(points={{-189,120},{-20,120},
          {-20,49},{-2,49}},  color={0,0,127}));
  connect(heaFlo.port, del.heatPort) annotation (Line(points={{80,0},{80,-10},{90,
          -10}},              color={191,0,0}));
  connect(hex.port_a2, senTRet.port_b) annotation (Line(points={{-108,-50},{-108,
          -60},{-90,-60}},
                      color={0,127,255}));
  connect(movMea.u, add2.y)
    annotation (Line(points={{108,140},{100,140},{100,120},{82,120}},
                                                 color={0,0,127}));
  connect(loa.y[2], add2.u1) annotation (Line(points={{-189,120},{40,120},{40,
          126},{58,126}},
                     color={0,0,127}));
  connect(temSupSet.y, masFlo.TSupSet) annotation (Line(points={{-188,60},{-120,
          60},{-120,40},{-10,40},{-10,43},{-2,43}},
                                    color={0,0,127}));
  connect(senTSup.T, masFlo.TSup_actual)
    annotation (Line(points={{-80,-9},{-80,37},{-2,37}},    color={0,0,127}));
  connect(pum.port_b, del.ports[1])
    annotation (Line(points={{50,-20},{97.3333,-20}},color={0,127,255}));
  connect(del.ports[2], hea.port_a)
    annotation (Line(points={{100,-20},{130,-20}},
                                               color={0,127,255}));
  connect(bou2.ports[1], del.ports[3]) annotation (Line(points={{100,-30},{100,
          -20},{102.667,-20}},
                         color={0,127,255}));
  connect(add2.y, int.u) annotation (Line(points={{82,120},{100,120},{100,100},
          {108,100}},
                 color={0,0,127}));
  connect(booleanExpression.y, int.trigger)
    annotation (Line(points={{101,60},{120,60},{120,88}}, color={255,0,255}));
  connect(senTRet.port_a, hea.port_b) annotation (Line(points={{-70,-60},{160,-60},
          {160,-20},{150,-20}},
                             color={0,127,255}));
  connect(realExpression.y, add2.u2) annotation (Line(points={{31,100},{40,100},
          {40,114},{58,114}},  color={0,0,127}));
  connect(booleanExpression.y, int1.trigger) annotation (Line(points={{101,60},
          {160,60},{160,68}},  color={255,0,255}));
  connect(loa.y[2], int1.u) annotation (Line(points={{-189,120},{-20,120},{-20,
          80},{148,80}},
                     color={0,0,127}));
  connect(int1.y, NMBE.u2) annotation (Line(points={{171,80},{180,80},{180,84},
          {188,84}},  color={0,0,127}));
  connect(int.y, NMBE.u1) annotation (Line(points={{131,100},{180,100},{180,96},
          {188,96}},  color={0,0,127}));
  connect(masFlo.Q_flow_residual, heaFlo.Q_flow)
    annotation (Line(points={{22,34},{80,34},{80,20}},  color={0,0,127}));
  connect(masFlo.Q_flow_actual, hea.u) annotation (Line(points={{22,40},{120,40},
          {120,-14},{128,-14}}, color={0,0,127}));
  connect(masFlo.m_flow, pum.m_flow_in)
    annotation (Line(points={{22,46},{40,46},{40,-8}},  color={0,0,127}));
  connect(senMasFlo.port_b, pum.port_a)
    annotation (Line(points={{-30,-20},{30,-20}}, color={0,127,255}));
  connect(senMasFlo.m_flow, masFlo.m_flow_actual) annotation (Line(points={{-40,-9},
          {-40,31.2},{-2,31.2}},      color={0,0,127}));
  connect(heaFlo1.port, del1.heatPort)
    annotation (Line(points={{80,-160},{80,-170},{90,-170}}, color={191,0,0}));
  connect(senTSup1.T, masFlo1.TSup_actual) annotation (Line(points={{-80,-169},{
          -80,-123},{-2,-123}}, color={0,0,127}));
  connect(masFlo1.Q_flow_residual, heaFlo1.Q_flow)
    annotation (Line(points={{22,-126},{80,-126},{80,-140}}, color={0,0,127}));
  connect(senTSup1.port_b, senMasFlo1.port_a)
    annotation (Line(points={{-70,-180},{-50,-180}}, color={0,127,255}));
  connect(senMasFlo1.m_flow, masFlo1.m_flow_actual) annotation (Line(points={{-40,
          -169},{-40,-128.8},{-2,-128.8}}, color={0,0,127}));
  connect(serSup1.ports[1], senTSup1.port_a)
    annotation (Line(points={{-190,-180},{-90,-180}}, color={0,127,255}));
  connect(senTRet1.port_b, serRet1.ports[1])
    annotation (Line(points={{-90,-220},{-190,-220}}, color={0,127,255}));
  connect(valPreInd.y, gai.y)
    annotation (Line(points={{40,-168},{40,-162}}, color={0,0,127}));
  connect(senTSup.port_b, senMasFlo.port_a)
    annotation (Line(points={{-70,-20},{-50,-20}}, color={0,127,255}));
  connect(loa.y[2], masFlo1.Q_flow) annotation (Line(points={{-189,120},{-20,120},
          {-20,-111},{-2,-111}}, color={0,0,127}));
  connect(temSupSet.y, masFlo1.TSupSet) annotation (Line(points={{-188,60},{-180,
          60},{-180,-120},{-20,-120},{-20,-117},{-2,-117}}, color={0,0,127}));
  connect(del1.ports[1], hea1.port_a)
    annotation (Line(points={{97.3333,-180},{130,-180}}, color={0,127,255}));
  connect(bou1.ports[1], del1.ports[2]) annotation (Line(points={{100,-190},{
          100,-186},{100,-180},{100,-180}}, color={0,127,255}));
  connect(senMasFlo1.port_b, valPreInd.port_a)
    annotation (Line(points={{-30,-180},{30,-180}}, color={0,127,255}));
  connect(valPreInd.port_b, del1.ports[3])
    annotation (Line(points={{50,-180},{102.667,-180}}, color={0,127,255}));
  connect(hea1.port_b, senTRet1.port_a) annotation (Line(points={{150,-180},{
          160,-180},{160,-220},{-70,-220}}, color={0,127,255}));
  connect(masFlo1.m_flow, gai.u)
    annotation (Line(points={{22,-114},{40,-114},{40,-138}}, color={0,0,127}));
  connect(masFlo1.Q_flow_actual, hea1.u) annotation (Line(points={{22,-120},{
          120,-120},{120,-174},{128,-174}}, color={0,0,127}));
  annotation (Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-260,-320},{260,160}})),
      experiment(StopTime=360000),
      __Dymola_Commands(
      file="modelica://Buildings/Resources/Scripts/Dymola/Experimental/DHC/Loads/Validation/EnergyMassFlow.mos"
      "Simulate and plot"),
    Documentation(info="<html>
<p>
This is a validation model for the block
Buildings.Experimental.DHC.Loads.EnergyMassFlow.
The service supply temperature is voluntarily lowered 
to simulate a mismatch between the supply temperature
set point of the building
distribution system, and the actual supply temperature. 
This leads to an increase in the mass flow rate to compensate
for the low supply temperature.
When the mass flow rate reaches its nominal value, a part of the 
load cannot be met.
This part is applied as a heat flow rate to a control volume.
It will contribute to the variation of the average temperature 
of the fluid volume inside the distribution system.
The part of the load that can be met is directly applied to the fluid
stream as a steady-state heat flow rate.
</p>
<p>
Changing the boundary condition for the service supply temperature allows
to simulate conditions where the load is always met and conditions where
the load cannot be met transiently.
In both cases the NMBE between the time series and the actual heat flow 
rate transferred by the ETS heat exchanger remains close to zero.
</p>
</html>"));
end EnergyMassFlow;
