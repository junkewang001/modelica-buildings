within Buildings.Controls.OBC.RooftopUnits.DXCoil.Subsequences.Validation;
model DXCoilEnable
  "Validate sequence for enabling and disabling DX coil using cooling coil valve postion signal"

  Buildings.Controls.OBC.RooftopUnits.DXCoil.Subsequences.DXCoilEnable DXCoiEna(
    nCoi=1,
    uThrCooCoi2=0.8,
    uThrCooCoi3=0.1,
    dUHys=0.01,
    timPer2=300,
    timPer3=300)
    "Enable DX coil"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Pulse pul(
    final amplitude=0.9,
    final width=0.6,
    final period=1800,
    final offset=0.05)
    "Cooling coil valve position"
    annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));

  Buildings.Controls.OBC.CDL.Logical.TrueFalseHold CoiEna(
    final trueHoldDuration=10)
    "Hold pulse signal for easy visualization"
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));

  Buildings.Controls.OBC.CDL.Logical.Pre pre
    "Logical pre block"
    annotation (Placement(transformation(extent={{40,30},{60,50}})));

equation
  connect(pul.y, DXCoiEna.uCooCoi) annotation (Line(points={{-38,-40},{-20,-40},
          {-20,-6},{-12,-6}}, color={0,0,127}));
  connect(DXCoiEna.yDXCoi, CoiEna.u)
    annotation (Line(points={{12,0},{38,0}}, color={255,0,255}));
  connect(DXCoiEna.yDXCoi, pre.u) annotation (Line(points={{12,0},{20,0},
          {20,40},{38,40}}, color={255,0,255}));
  connect(pre.y, DXCoiEna.uDXCoi[1]) annotation (Line(points={{62,40},{70,
          40},{70,60},{-20,60},{-20,6},{-12,6}}, color={255,0,255}));
annotation (
  experiment(StopTime=3600.0, Tolerance=1e-06),
  __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Controls/OBC/RooftopUnits/DXCoil/Subsequences/Validation/DXCoilEnable.mos"
    "Simulate and plot"),
  Documentation(info="<html>
<p>
This example validates
<a href=\"modelica://Buildings.Controls.OBC.RooftopUnits.DXCoil.Subsequences.Validation.DXCoilEnable\">
Buildings.Controls.OBC.RooftopUnits.DXCoil.Subsequences.Validation.DXCoilEnable</a>.
</p>
</html>", revisions="<html>
<ul>
<li>
July 13, 2022, by Junke Wang and Karthik Devaprasad:<br/>
First implementation.
</li>
</ul>
</html>"),
  Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
    graphics={
      Ellipse(lineColor = {75,138,73},
              fillColor={255,255,255},
              fillPattern = FillPattern.Solid,
              extent = {{-100,-100},{100,100}}),
      Polygon(lineColor = {0,0,255},
              fillColor = {75,138,73},
              pattern = LinePattern.None,
              fillPattern = FillPattern.Solid,
              points = {{-36,60},{64,0},{-36,-60},{-36,60}})}),
  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
end DXCoilEnable;
