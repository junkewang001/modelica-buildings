within Buildings.Fluid.HeatPumpWaterHeaters;
model WrappedCondenser
  extends Buildings.Fluid.Interfaces.PartialFourPortInterface
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
  Movers.FlowControlled_m_flow mov
    annotation (Placement(transformation(extent={{50,50},{70,70}})));
  Storage.StratifiedEnhanced tan
    annotation (Placement(transformation(extent={{60,-70},{80,-50}})));
  DXSystems.Cooling.AirSource.SingleSpeed sinSpeDXCoo
    annotation (Placement(transformation(extent={{-42,50},{-22,70}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow hea
    "Heat input at the bottom of the tank"
    annotation (Placement(transformation(extent={{30,-70},{50,-50}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TTop
    "Temperature tank top"
    annotation (Placement(transformation(extent={{-60,-50},{-80,-30}})));
  Modelica.Blocks.Logical.Hysteresis onOffHea(uLow=273.15 + 50 - 0.05, uHigh=
        273.15 + 50 + 0.05) "Controller for heater at middle"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Modelica.Blocks.Math.BooleanToReal yMid
    "Boolean to real conversion for valve at middle"
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Controls.OBC.CDL.Reals.Add add2
    annotation (Placement(transformation(extent={{10,10},{30,30}})));
  Controls.OBC.CDL.Reals.Multiply mul
    annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
equation
  connect(port_b2, tan.port_b) annotation (Line(points={{-100,-60},{-80,-60},{
          -80,-80},{70,-80},{70,-70}}, color={0,127,255}));
  connect(port_a2, tan.port_a) annotation (Line(points={{100,-60},{88,-60},{88,
          -40},{70,-40},{70,-50}}, color={0,127,255}));
  connect(sinSpeDXCoo.port_a, port_a1)
    annotation (Line(points={{-42,60},{-100,60}}, color={0,127,255}));
  connect(sinSpeDXCoo.port_b, mov.port_a)
    annotation (Line(points={{-22,60},{50,60}}, color={0,127,255}));
  connect(mov.port_b, port_b1)
    annotation (Line(points={{70,60},{100,60}}, color={0,127,255}));
  connect(hea.port, tan.heaPorVol[5]) annotation (Line(points={{50,-60},{70,-60}},
                              color={191,0,0}));
  connect(TTop.port, tan.heaPorVol[1]) annotation (Line(points={{-60,-40},{60,
          -40},{60,-60},{70,-60}},
                             color={191,0,0}));
  connect(onOffHea.u,TTop. T) annotation (Line(points={{-82,0},{-90,0},{-90,-40},
          {-81,-40}},            color={0,0,127}));
  connect(sinSpeDXCoo.on, onOffHea.y) annotation (Line(points={{-43,68},{-50,68},
          {-50,0},{-59,0}}, color={255,0,255}));
  connect(onOffHea.y, yMid.u)
    annotation (Line(points={{-59,0},{-42,0}}, color={255,0,255}));
  connect(yMid.y, mov.m_flow_in) annotation (Line(points={{-19,0},{0,0},{0,80},
          {60,80},{60,72}}, color={0,0,127}));
  connect(sinSpeDXCoo.QSen_flow, add2.u1) annotation (Line(points={{-21,67},{-8,
          67},{-8,26},{8,26}}, color={0,0,127}));
  connect(sinSpeDXCoo.QLat_flow, add2.u2) annotation (Line(points={{-21,65},{
          -21,64},{-14,64},{-14,14},{8,14}}, color={0,0,127}));
  connect(mul.y, hea.Q_flow)
    annotation (Line(points={{22,-60},{30,-60}}, color={0,0,127}));
  connect(yMid.y, mul.u2) annotation (Line(points={{-19,0},{-12,0},{-12,-66},{
          -2,-66}}, color={0,0,127}));
  connect(add2.y, mul.u1) annotation (Line(points={{32,20},{40,20},{40,-32},{-6,
          -32},{-6,-54},{-2,-54}}, color={0,0,127}));
end WrappedCondenser;
