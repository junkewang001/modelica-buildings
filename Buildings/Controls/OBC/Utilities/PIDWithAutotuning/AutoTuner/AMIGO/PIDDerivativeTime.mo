within Buildings.Controls.OBC.Utilities.PIDWithAutotuning.AutoTuner.Amigo;
block PIDDerivativeTime "Identifies the derivative time of a PID controller"
  Buildings.Controls.OBC.CDL.Interfaces.RealInput T(min=0)
    "Connector for the signal of the time constant of a first order time-delayed model"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}}),
        iconTransformation(extent={{-140,40},{-100,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput L(min=1E-6)
    "Connector for the signal of the time delay of a first order time-delayed model"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}}),
        iconTransformation(extent={{-140,-80},{-100,-40}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput Td
    "Connector for time constant signal for the derivative term"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  CDL.Continuous.MultiplyByParameter gai1(k=0.3) "0.3L"
    annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
  CDL.Continuous.MultiplyByParameter gai2(k=0.5) "0.5T"
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
  CDL.Continuous.Multiply mul "0.5LT"
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  CDL.Continuous.Add add "0.3L+T"
    annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
  CDL.Continuous.Divide div "0.5LT/(0.3L+T)"
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
equation
  connect(div.y, Td)
    annotation (Line(points={{42,0},{110,0}}, color={0,0,127}));
  connect(gai1.u, L) annotation (Line(points={{-82,-40},{-96,-40},{-96,-60},{
          -120,-60}}, color={0,0,127}));
  connect(gai1.y, add.u2) annotation (Line(points={{-58,-40},{-50,-40},{-50,-56},
          {-42,-56}}, color={0,0,127}));
  connect(add.u1, T) annotation (Line(points={{-42,-44},{-48,-44},{-48,60},{
          -120,60}}, color={0,0,127}));
  connect(gai2.u, T) annotation (Line(points={{-82,40},{-90,40},{-90,60},{-120,
          60}}, color={0,0,127}));
  connect(mul.u2, L) annotation (Line(points={{-42,4},{-96,4},{-96,-60},{-120,
          -60}}, color={0,0,127}));
  connect(gai2.y, mul.u1) annotation (Line(points={{-58,40},{-52,40},{-52,16},{
          -42,16}}, color={0,0,127}));
  connect(mul.y, div.u1) annotation (Line(points={{-18,10},{10,10},{10,6},{18,6}},
        color={0,0,127}));
  connect(div.u2, add.y) annotation (Line(points={{18,-6},{12,-6},{12,-50},{-18,
          -50}}, color={0,0,127}));
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
<p>This block calculates the derivative time of a PID model, <i>T<sub>d</sub></i>, by</p>
<p>T<sub>d</sub> = 0.5LT/(0.3L+T) </p>
<p>where <i>T</i> is the time constant of the first-order time-delayed model;</p>
<p><i>L</i> is the time delay of the first-order time-delayed model.</p>
<h4>References</h4>
<p>&Aring;str&ouml;m, Karl Johan, and Tore H&auml;gglund. &quot;Revisiting the Ziegler&ndash;Nichols step response method for PID control.&quot; Journal of process control 14.6 (2004): 635-650.</p>
</html>"));
end PIDDerivativeTime;
