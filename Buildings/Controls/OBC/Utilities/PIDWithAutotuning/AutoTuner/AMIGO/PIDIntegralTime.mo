within Buildings.Controls.OBC.Utilities.PIDWithAutotuning.AutoTuner.Amigo;
block PIDIntegralTime "Identifies the integral time of a PID controller"
  Buildings.Controls.OBC.CDL.Interfaces.RealInput T(min=0)
    "Connector for the signal of the time constant of a first order time-delayed model"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}}),
        iconTransformation(extent={{-140,40},{-100,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput L(min=1E-6)
    "Connector for the signal of the time delay of a first order time-delayed model"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}}),
        iconTransformation(extent={{-140,-80},{-100,-40}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Ti
    "Connector for time constant signal for the integral term"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  CDL.Continuous.MultiplyByParameter gai1(k=0.4) "0.4L"
    annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
  CDL.Continuous.MultiplyByParameter gai2(k=0.8) "0.8T"
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
  CDL.Continuous.MultiplyByParameter gai3(k=0.1) "0.1T"
    annotation (Placement(transformation(extent={{-80,68},{-60,88}})));
  CDL.Continuous.Add add1 "L+0.1T"
    annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
  CDL.Continuous.Add add2 "0.4L+0.8T"
    annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
  CDL.Continuous.Divide div3 "(0.4L+0.8T)/(L+0.1T)"
    annotation (Placement(transformation(extent={{0,10},{20,-10}})));
  CDL.Continuous.Multiply mul1 "(0.4L+0.8T)/(L+0.1T)L"
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
equation
  connect(gai1.u, L) annotation (Line(points={{-82,-40},{-90,-40},{-90,-60},{
          -120,-60}}, color={0,0,127}));
  connect(gai3.u, T) annotation (Line(points={{-82,78},{-90,78},{-90,60},{-120,
          60}}, color={0,0,127}));
  connect(gai2.u, T) annotation (Line(points={{-82,40},{-90,40},{-90,60},{-120,
          60}}, color={0,0,127}));
  connect(gai1.y, add2.u2) annotation (Line(points={{-58,-40},{-48,-40},{-48,
          -26},{-42,-26}}, color={0,0,127}));
  connect(add2.u1, gai2.y) annotation (Line(points={{-42,-14},{-48,-14},{-48,40},
          {-58,40}}, color={0,0,127}));
  connect(add1.u2, L) annotation (Line(points={{-42,14},{-54,14},{-54,-60},{
          -120,-60}}, color={0,0,127}));
  connect(add1.u1, gai3.y) annotation (Line(points={{-42,26},{-54,26},{-54,78},
          {-58,78}}, color={0,0,127}));
  connect(div3.y, mul1.u1)
    annotation (Line(points={{22,0},{30,0},{30,6},{38,6}}, color={0,0,127}));
  connect(mul1.u2, L) annotation (Line(points={{38,-6},{24,-6},{24,-60},{-120,
          -60}}, color={0,0,127}));
  connect(mul1.y, Ti)
    annotation (Line(points={{62,0},{110,0}}, color={0,0,127}));
  connect(div3.u2, add1.y) annotation (Line(points={{-2,6},{-10,6},{-10,20},{
          -18,20}}, color={0,0,127}));
  connect(div3.u1, add2.y) annotation (Line(points={{-2,-6},{-10,-6},{-10,-20},
          {-18,-20}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,-100},{100,100}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-154,148},{146,108}},
          textString="%name",
          textColor={0,0,255})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(revisions="<html>
<ul>
<li>
June 1, 2022, by Sen Huang:<br/>
First implementation<br/>
</li>
</ul>
</html>", info="<html>
<p>This block calculates the integral time of a PID model, <i>T<sub>i</sub></i>, by</p>
<p>T<sub>i</sub> = (0.4L + 0.8T)/(L + 0.1T)L </p>
<p>where <i>T</i> is the time constant of the first-order time-delayed model;</p>
<p><i>L</i> is the time delay of the first-order time-delayed model.</p>
<h4>References</h4>
<p>&Aring;str&ouml;m, Karl Johan, and Tore H&auml;gglund. &quot;Revisiting the Ziegler&ndash;Nichols step response method for PID control.&quot; Journal of process control 14.6 (2004): 635-650.</p>
</html>"));
end PIDIntegralTime;
