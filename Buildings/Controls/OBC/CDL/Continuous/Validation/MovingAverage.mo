within Buildings.Controls.OBC.CDL.Continuous.Validation;
model MovingAverage "Validation model for the MovingAverage block"
  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sinInpNoDel(
    freqHz=1/80) "Start from zero second"
    annotation (Placement(transformation(extent={{-90,50},{-70,70}})));
  Buildings.Controls.OBC.CDL.Continuous.MovingAverage movAve1(delta=100)
    "Moving average with 100 s sliding window"
    annotation (Placement(transformation(extent={{-40,50},{-20,70}})));
  Buildings.Controls.OBC.CDL.Continuous.MovingAverage movAve2(delta=200)
    "Moving average with 200 s sliding window"
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  Buildings.Controls.OBC.CDL.Continuous.MovingAverage movAve3(delta=300)
    "Moving average with 300 s sliding window"
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Buildings.Controls.OBC.CDL.Continuous.MovingAverage movAve4(delta=400)
    "Moving average with 400 s sliding window"
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
  Buildings.Controls.OBC.CDL.Continuous.MovingAverage movAve5(delta=500)
    "Moving average with 500 s sliding window"
    annotation (Placement(transformation(extent={{-40,-70},{-20,-50}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sinInp50sDel(
    freqHz=1/80,
    startTime=50)
    "Start from 50 seconds"
    annotation (Placement(transformation(extent={{0,50},{20,70}})));
  Buildings.Controls.OBC.CDL.Continuous.MovingAverage movAve6(delta=100)
    "Moving average with 100 s sliding window"
    annotation (Placement(transformation(extent={{60,50},{80,70}})));
  Buildings.Controls.OBC.CDL.Continuous.MovingAverage movAve7(delta=200)
    "Moving average with 200 s sliding window"
    annotation (Placement(transformation(extent={{60,20},{80,40}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sinInp100sDel(
    freqHz=1/80,
    startTime=100)
    "Start from 100 seconds"
    annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
  Buildings.Controls.OBC.CDL.Continuous.MovingAverage movAve8(delta=100)
    "Moving average with 100 s sliding window"
    annotation (Placement(transformation(extent={{60,-30},{80,-10}})));
  Buildings.Controls.OBC.CDL.Continuous.MovingAverage movAve9(delta=200)
    "Moving average with 200 s sliding window"
    annotation (Placement(transformation(extent={{60,-60},{80,-40}})));

equation
  connect(sinInpNoDel.y,movAve3.u)
    annotation (Line(points={{-69,60},{-60,60},{-60,0},{-42,0}},color={0,0,127}));
  connect(sinInpNoDel.y,movAve1.u)
    annotation (Line(points={{-69,60},{-42,60}},color={0,0,127}));
  connect(sinInpNoDel.y,movAve2.u)
    annotation (Line(points={{-69,60},{-60,60},{-60,30},{-42,30}},color={0,0,127}));
  connect(sinInpNoDel.y,movAve5.u)
    annotation (Line(points={{-69,60},{-60,60},{-60,-60},{-42,-60}},color={0,0,127}));
  connect(sinInpNoDel.y,movAve4.u)
    annotation (Line(points={{-69,60},{-60,60},{-60,-30},{-42,-30}},color={0,0,127}));
  connect(sinInp50sDel.y,movAve6.u)
    annotation (Line(points={{21,60},{21,60},{58,60}},color={0,0,127}));
  connect(sinInp50sDel.y,movAve7.u)
    annotation (Line(points={{21,60},{21,60},{40,60},{40,30},{58,30}},color={0,0,127}));
  connect(sinInp100sDel.y,movAve9.u)
    annotation (Line(points={{21,-20},{21,-20},{40,-20},{40,-50},{58,-50}},color={0,0,127}));
  connect(sinInp100sDel.y,movAve8.u)
    annotation (Line(points={{21,-20},{58,-20}},color={0,0,127}));
  annotation (
    experiment(
      StopTime=900.0,
      Tolerance=1e-06),
    __Dymola_Commands(
      file="modelica://Buildings/Resources/Scripts/Dymola/Controls/OBC/CDL/Continuous/Validation/MovingAverage.mos" "Simulate and plot"),
    Documentation(
      info="<html>
<p>
Validation test for the block
<a href=\"modelica://Buildings.Controls.OBC.CDL.Continuous.MovingAverage\">
Buildings.Controls.OBC.CDL.Continuous.MovingAverage</a>.
</p>
<p>
The input <code>sinInpNoDel</code>,  <code>sinInp50sDel</code>,
<code>sinInp100sDel</code>,  generate sine outputs with same frequency of
<code>1/80 Hz</code>, but different start times of <code>0 second</code>,
<code>50 second</code>, <code>100 second</code>.
</p>
</html>",
revisions="<html>
<ul>
<li>
March 17, 2023, by Michael Wetter:<br/>
Replaced blocks from Modelica Standard Library with CDL blocks.
</li>
<li>
January 27, 2022, by Jianjun Hu:<br/>
Renamed the block name from MovingMean to MovingAverage.<br/>
This is for <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2865\">issue 2865</a>.
</li>
<li>
June 29, 2017, by Jianjun Hu:<br/>
First implementation.
</li>
</ul>
</html>"),
    Icon(
      graphics={
        Ellipse(
          lineColor={75,138,73},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          extent={{-100,-100},{100,100}}),
        Polygon(
          lineColor={0,0,255},
          fillColor={75,138,73},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-36,60},{64,0},{-36,-60},{-36,60}})}));
end MovingAverage;
