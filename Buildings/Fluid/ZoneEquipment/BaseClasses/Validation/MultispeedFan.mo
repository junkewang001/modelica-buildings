within Buildings.Fluid.ZoneEquipment.BaseClasses.Validation;
model MultispeedFan
  "Validation model for multiple speed fan controller"
  extends Modelica.Icons.Example;

  Buildings.Fluid.ZoneEquipment.BaseClasses.MultispeedFan conMulSpeFanConWat(
    has_hea=true,
    has_coo=true)
    "Instance of controller with multiple speed fan"
    annotation (Placement(transformation(extent={{28,-14},{56,14}})));

  Buildings.Controls.OBC.CDL.Logical.Sources.Pulse uAva(
    final period=21600,
    shift=10800)
    "Availability signal"
    annotation (Placement(transformation(extent={{-40,-50},{-20,-30}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Ramp TZon(
    final height=12,
    final duration=36000,
    final offset=273.15 + 16)
    "Measured zone temperature"
    annotation (Placement(transformation(extent={{-40,70},{-20,90}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant heaSetPoi(
    final k=273.15 + 21)
    "Heating setpoint temperature"
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));

  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant cooSetPoi(
    final k=273.15 + 23)
    "Cooling setpoint temperature"
    annotation (Placement(transformation(extent={{-40,30},{-20,50}})));

  Controls.OBC.CDL.Logical.Sources.Pulse fanOpeMod(
    final period=21600)
    "Fan opearating mode"
    annotation (Placement(transformation(extent={{-40,-88},{-20,-68}})));

equation
  connect(conMulSpeFanConWat.TZon, TZon.y) annotation (Line(points={{26,12},{10,
          12},{10,80},{-18,80}}, color={0,0,127}));
  connect(conMulSpeFanConWat.TCooSet, cooSetPoi.y)
    annotation (Line(points={{26,6},{0,6},{0,40},{-18,40}}, color={0,0,127}));
  connect(conMulSpeFanConWat.THeaSet, heaSetPoi.y)
    annotation (Line(points={{26,0},{-18,0}}, color={0,0,127}));
  connect(conMulSpeFanConWat.uAva, uAva.y) annotation (Line(points={{26,-6},{0,-6},
          {0,-40},{-18,-40}}, color={255,0,255}));
  connect(fanOpeMod.y, conMulSpeFanConWat.fanOpeMod) annotation (Line(points={{-18,
          -78},{10,-78},{10,-12},{26,-12}}, color={255,0,255}));
  annotation(Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
    <p>
    This simulation model is used to validate 
    <a href=\"modelica://Buildings.Fluid.ZoneEquipment.BaseClasses.MultispeedFan\">
    Buildings.Fluid.ZoneEquipment.BaseClasses.MultispeedFan</a>. 
    </p>
    </html>",revisions="<html>
      <ul>
      <li>
      June 20, 2023, by Junke Wang and Karthik Devaprasad:<br/>
      First implementation.
      </li>
      </ul>
      </html>"),
    experiment(Tolerance=1e-06),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/ZoneEquipment/BaseClasses/Validation/MultispeedFan.mos"
        "Simulate and Plot"));
end MultispeedFan;
