within Buildings.Fluid.Storage.Plant.Examples;
model OpenDecoupled
  "A two-source three-user network with an open tank, pressure is decoupled"
  extends Modelica.Icons.Example;
  extends Buildings.Fluid.Storage.Plant.Examples.BaseClasses.PartialDualSource;

  Buildings.Fluid.Storage.Plant.IdealReversibleConnection ideRevConRet(
    redeclare final package Medium = Medium,
    final m_flow_nominal=nom.mTan_flow_nominal) "Ideal reversable connection on supply side"
    annotation (Placement(transformation(extent={{0,-120},{20,-100}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiplyByParameter gai(final k=-1)
    "Take additive inverse" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,-90})));
  Buildings.Fluid.Sources.Boundary_pT bou1(
    redeclare final package Medium = Medium,
    p(displayUnit="Pa") = 101325 + dp_nominal,
    nPorts=1) "Pressure boundary" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-70,90})));
  Buildings.Fluid.Sources.Boundary_pT bou2(
    redeclare final package Medium = Medium,
    p(displayUnit="Pa") = 101325 + dp_nominal,
    nPorts=1) "Pressure boundary" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-70,10})));
equation
  connect(gai.y, ideRevConRet.mSet_flow) annotation (Line(points={{-18,-90},{-10,
          -90},{-10,-105},{-1,-105}}, color={0,0,127}));
  connect(ideRevConRet.port_b, parJunPla2.port_c2) annotation (Line(points={{20,
          -110},{30,-110},{30,-96},{40,-96}}, color={0,127,255}));
  connect(tanBra.port_aRetNet, ideRevConRet.port_a) annotation (Line(points={{-80,
          -96},{-70,-96},{-70,-110},{0,-110}}, color={0,127,255}));
  connect(bou1.ports[1], pumSup1.port_a)
    annotation (Line(points={{-60,90},{-20,90}}, color={0,127,255}));
  connect(bou2.ports[1], ideRevConSup.port_a) annotation (Line(points={{-60,10},
          {-10,10},{-10,-70},{0,-70}}, color={0,127,255}));
  connect(gai.u, floCon.mSecPum_flow) annotation (Line(points={{-42,-90},{-50,
          -90},{-50,-34},{-139,-34}}, color={0,0,127}));
  annotation (experiment(Tolerance=1e-06, StopTime=9000),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Plant/Examples/OpenDecoupled.mos"
        "Simulate and plot"),Diagram(coordinateSystem(extent={{-280,-240},{220,220}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})),
        Documentation(info="<html>
<p>
This variant of the two-plant-three-user example model represents the scenario
where the storage plant is open and the pressure is decoupled.
This means the district system has one pressurisation point at the chiller-only plant.
Since the open tank at the storage plant forces the storage plant also to have
a pressurisation point, 
the return side of the storage plant is connected to the network with
a pump-valve connection to decouple its pressure from the network.
</p>
</html>", revisions="<html>
<ul>
<li>
January 11, 2023 by Hongxiang Fu:<br/>
First implementation. This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2859\">#2859</a>.
</li>
</ul>
</html>"));
end OpenDecoupled;
