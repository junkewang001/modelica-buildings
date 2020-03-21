within Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences;
block Up "Generates a stage up signal"
  parameter Boolean have_WSE = true
    "true = plant has a WSE, false = plant does not have WSE";

  parameter Modelica.SIunits.Time delayStaCha = 15*60
    "Delay stage change";

  parameter Modelica.SIunits.Time shortDelay = 10*60
    "Short stage 0 to 1 delay";

  parameter Modelica.SIunits.Time longDelay = 20*60
    "Long stage 0 to 1 delay";

  parameter Modelica.SIunits.Time upHolPer = 900
     "Time period for the value hold at stage up change";

  parameter Modelica.SIunits.TemperatureDifference smallTDif = 1
    "Offset between the chilled water supply temperature and its setpoint";

  parameter Modelica.SIunits.TemperatureDifference largeTDif = 2
    "Offset between the chilled water supply temperature and its setpoint";

  parameter Modelica.SIunits.TemperatureDifference TDif = 1
    "Offset between the chilled water supply temperature and its setpoint";

  parameter Modelica.SIunits.PressureDifference dpDif = 2 * 6895
    "Offset between the chilled water pump Diferential static pressure and its setpoint";

  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput u if       have_WSE
    "Chiller stage" annotation (Placement(transformation(extent={{-200,-180},{-160,
            -140}}),iconTransformation(extent={{-140,-80},{-100,-40}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uOpe(final unit="1")
    "Operating part load ratio of the current stage" annotation (Placement(
        transformation(extent={{-200,80},{-160,120}}),  iconTransformation(
          extent={{-140,80},{-100,120}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uStaUp(final unit="1")
    "Staging part load ratio of the next stage up" annotation (Placement(
        transformation(extent={{-200,50},{-160,90}}),   iconTransformation(
          extent={{-140,60},{-100,100}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dpChiWatPumSet(
    final unit="Pa",
    final quantity="PressureDifference")
    "Chilled water pump Diferential static pressure setpoint"
    annotation (Placement(transformation(extent={{-200,0},{-160,40}}),
      iconTransformation(extent={{-140,-20},{-100,20}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dpChiWatPum(
    final unit="Pa",
    final quantity="PressureDifference")
    "Chilled water pump Diferential static pressure"
    annotation (Placement(transformation(extent={{-200,-30},{-160,10}}),
    iconTransformation(extent={{-140,-40},{-100,0}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TChiWatSupSet(
    final unit="K",
    final quantity="ThermodynamicTemperature")
    "Chilled water supply temperature setpoint"
    annotation (Placement(transformation(extent={{-200,-110},{-160,-70}}),
    iconTransformation(extent={{-140,30},{-100,70}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TChiWatSup(
    final unit="K",
    final quantity="ThermodynamicTemperature")
    "Chilled water return temperature"
    annotation (Placement(transformation(extent={{-200,-150},{-160,-110}}),
    iconTransformation(extent={{-140,10},{-100,50}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput y
    "Stage up signal"
    annotation (Placement(transformation(extent={{160,-20},{200,20}}),
      iconTransformation(extent={{100,-20},{140,20}})));

//protected
  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.FailsafeCondition faiSafCon(
    final delayStaCha = delayStaCha,
    final TDif = TDif,
    final dpDif = dpDif)
    "Failsafe condition of the current stage"
    annotation (Placement(transformation(extent={{-100,10},{-80,30}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.EfficiencyCondition effCon(
    final delayStaCha = delayStaCha)
    "Efficiency condition of the current stage"
    annotation (Placement(transformation(extent={{-100,70},{-80,90}})));

  Buildings.Controls.OBC.CDL.Logical.Or orStaUp "Or for staging up"
    annotation (Placement(transformation(extent={{-20,30},{0,50}})));

  Buildings.Controls.OBC.CDL.Logical.LogicalSwitch logSwi
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));

  Buildings.Controls.OBC.CDL.Integers.GreaterThreshold intGreThr if have_WSE
    "Switches staging up rules"
    annotation (Placement(transformation(extent={{-100,-170},{-80,-150}})));

  Buildings.Controls.OBC.CDL.Continuous.Hysteresis hysTSup(
    final uLow=smallTDif - 1,
    final uHigh=smallTDif,
    final pre_y_start=false) if have_WSE
    "Checks if the chilled water supply temperature is higher than its setpoint plus an offset"
    annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));

  Buildings.Controls.OBC.CDL.Continuous.Hysteresis hysTSup1(
    final uLow=largeTDif - 1,
    final uHigh=largeTDif,
    final pre_y_start=false) if have_WSE
    "Checks if the chilled water supply temperature is higher than its setpoint plus an offset"
    annotation (Placement(transformation(extent={{-60,-110},{-40,-90}})));

  Buildings.Controls.OBC.CDL.Logical.Or orStaUp1 if have_WSE "Or for staging up"
    annotation (Placement(transformation(extent={{40,-90},{60,-70}})));

  CDL.Continuous.Feedback                   add0 if
                   have_WSE
    "Adder for temperatures"
    annotation (Placement(transformation(extent={{-110,-100},{-90,-80}})));

  Buildings.Controls.OBC.CDL.Logical.TrueDelay truDel(
    final delayTime=longDelay, delayOnInit=true) if have_WSE
    "Delays a true signal"
    annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));

  Buildings.Controls.OBC.CDL.Logical.TrueDelay truDel1(
    final delayTime=shortDelay, delayOnInit=true) if have_WSE
    "Delays a true signal"
    annotation (Placement(transformation(extent={{-20,-110},{0,-90}})));

  Buildings.Controls.OBC.CDL.Logical.Sources.Constant noWSE(final k=true) if not have_WSE
    "Replacement signal if plant does not have WSE - assuming if plant gets enabled the lowest available stage should be engaged"
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

  CDL.Interfaces.BooleanInput                        uAvaCur
    "Current stage availability status"
    annotation (Placement(transformation(extent={{-200,-60},{-160,-20}}),
        iconTransformation(extent={{-140,-110},{-100,-70}})));
equation
  connect(uOpe, effCon.uOpe) annotation (Line(points={{-180,100},{-150,100},{-150,
          85},{-102,85}},  color={0,0,127}));
  connect(uStaUp, effCon.uStaUp) annotation (Line(points={{-180,70},{-150,70},{-150,
          75},{-102,75}},        color={0,0,127}));
  connect(TChiWatSupSet, faiSafCon.TChiWatSupSet) annotation (Line(points={{-180,
          -90},{-150,-90},{-150,27},{-102,27}},
                                              color={0,0,127}));
  connect(TChiWatSup, faiSafCon.TChiWatSup) annotation (Line(points={{-180,-130},
          {-130,-130},{-130,23},{-102,23}},color={0,0,127}));
  connect(dpChiWatPumSet, faiSafCon.dpChiWatPumSet) annotation (Line(points={{-180,20},
          {-102,20}},                 color={0,0,127}));
  connect(dpChiWatPum, faiSafCon.dpChiWatPum) annotation (Line(points={{-180,-10},
          {-110,-10},{-110,17},{-102,17}},  color={0,0,127}));
  connect(effCon.y, orStaUp.u1) annotation (Line(points={{-78,80},{-40,80},{-40,
          40},{-22,40}},   color={255,0,255}));
  connect(faiSafCon.y, orStaUp.u2) annotation (Line(points={{-78,20},{-50,20},{-50,
          32},{-22,32}},   color={255,0,255}));
  connect(intGreThr.y, logSwi.u2) annotation (Line(points={{-78,-160},{20,-160},
          {20,0},{78,0}},   color={255,0,255}));
  connect(orStaUp.y, logSwi.u1) annotation (Line(points={{2,40},{10,40},{10,8},{
          78,8}},  color={255,0,255}));
  connect(add0.y, hysTSup1.u)
    annotation (Line(points={{-88,-90},{-80,-90},{-80,-100},{-62,-100}},
                                                   color={0,0,127}));
  connect(hysTSup.y, truDel.u)
    annotation (Line(points={{-38,-60},{-22,-60}},color={255,0,255}));
  connect(hysTSup1.y, truDel1.u)
    annotation (Line(points={{-38,-100},{-22,-100}},
                                                  color={255,0,255}));
  connect(truDel.y, orStaUp1.u1) annotation (Line(points={{2,-60},{10,-60},{10,-80},
          {38,-80}},      color={255,0,255}));
  connect(truDel1.y, orStaUp1.u2) annotation (Line(points={{2,-100},{10,-100},{10,
          -88},{38,-88}},   color={255,0,255}));
  connect(orStaUp1.y, logSwi.u3) annotation (Line(points={{62,-80},{70,-80},{70,
          -8},{78,-8}},                  color={255,0,255}));
  connect(noWSE.y, logSwi.u2)
    annotation (Line(points={{2,0},{78,0}},    color={255,0,255}));
  connect(noWSE.y, logSwi.u3) annotation (Line(points={{2,0},{30,0},{30,-8},{78,
          -8}},    color={255,0,255}));
  connect(u, intGreThr.u)
    annotation (Line(points={{-180,-160},{-102,-160}},
                                                     color={255,127,0}));
  connect(uAvaCur, faiSafCon.uAvaCur) annotation (Line(points={{-180,-40},{-120,
          -40},{-120,13},{-102,13}},       color={255,0,255}));
  connect(logSwi.y, y)
    annotation (Line(points={{102,0},{180,0}}, color={255,0,255}));
  connect(add0.y, hysTSup.u) annotation (Line(points={{-88,-90},{-80,-90},{-80,
          -60},{-62,-60}}, color={0,0,127}));
  connect(TChiWatSup, add0.u1) annotation (Line(points={{-180,-130},{-130,-130},
          {-130,-90},{-112,-90}}, color={0,0,127}));
  connect(TChiWatSupSet, add0.u2) annotation (Line(points={{-180,-90},{-150,-90},
          {-150,-112},{-100,-112},{-100,-102}}, color={0,0,127}));
  annotation (defaultComponentName = "staUp",
        Icon(graphics={
        Rectangle(
        extent={{-100,-100},{100,100}},
        lineColor={0,0,127},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
        Text(
          extent={{-120,146},{100,108}},
          lineColor={0,0,255},
          textString="%name"),
        Rectangle(extent={{-80,-10},{-20,-22}}, lineColor={0,0,127}),
        Rectangle(extent={{-80,-28},{-20,-40}}, lineColor={0,0,127}),
        Rectangle(extent={{-76,-22},{-72,-28}}, lineColor={0,0,127}),
        Rectangle(extent={{-28,-22},{-24,-28}}, lineColor={0,0,127}),
        Rectangle(extent={{20,-10},{80,-22}}, lineColor={0,0,127}),
        Rectangle(extent={{20,-28},{80,-40}}, lineColor={0,0,127}),
        Rectangle(extent={{24,-22},{28,-28}}, lineColor={0,0,127}),
        Rectangle(extent={{72,-22},{76,-28}}, lineColor={0,0,127}),
        Rectangle(extent={{20,30},{80,18}}, lineColor={0,0,127}),
        Rectangle(extent={{20,12},{80,0}}, lineColor={0,0,127}),
        Rectangle(extent={{24,18},{28,12}}, lineColor={0,0,127}),
        Rectangle(extent={{72,18},{76,12}}, lineColor={0,0,127}),
        Line(points={{130,-48}}, color={0,0,127})}), Diagram(
        coordinateSystem(preserveAspectRatio=false,
        extent={{-160,-200},{160,140}})),
Documentation(info="<html>
<p>
Outputs a boolean stage up signal <code>y<\code> based on the various plant operation 
conditions that get provided as input signals. The implementation is according to 
ASHRAE RP1711 section 5.2.4.13 
July Draft.
</p>
</html>",
revisions="<html>
<ul>
<li>
January 28, 2019, by Milica Grahovac:<br/>
First implementation.
</li>
</ul>
</html>"));
end Up;