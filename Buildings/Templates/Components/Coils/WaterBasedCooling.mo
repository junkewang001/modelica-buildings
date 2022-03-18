within Buildings.Templates.Components.Coils;
model WaterBasedCooling "Chilled water coil"
  extends Buildings.Templates.Components.Coils.Interfaces.PartialCoil(
    final typ=Buildings.Templates.Components.Types.Coil.WaterBasedCooling,
    final typVal=val.typ,
    port_aSou(redeclare final package Medium = MediumChiWat),
    port_bSou(redeclare final package Medium = MediumChiWat));

  replaceable package MediumChiWat=Buildings.Media.Water
    "Source side medium";

  final parameter Modelica.Units.SI.MassFlowRate mWat_flow_nominal=
    dat.mWat_flow_nominal
    "Liquid mass flow rate";
  final parameter Modelica.Units.SI.PressureDifference dpWat_nominal=
    dat.dpWat_nominal
    "Liquid pressure drop across coil";
  final parameter Modelica.Units.SI.PressureDifference dpValve_nominal=
    dat.dpValve_nominal
    "Nominal pressure drop across fully open valve";

  parameter Buildings.Fluid.Types.HeatExchangerConfiguration configuration=
    Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow
    "Heat exchanger configuration"
    annotation (Evaluate=true);

  replaceable Buildings.Templates.Components.Valves.None val constrainedby
    Buildings.Templates.Components.Valves.Interfaces.PartialValve(redeclare
      final package Medium = MediumChiWat,
      final dat=datVal)
    "Valve"
    annotation (
      choices(
        choice(redeclare replaceable Buildings.Templates.Components.Valves.None val
          "No valve"),
        choice(redeclare replaceable Buildings.Templates.Components.Valves.ThreeWayModulating val
          "Three-way modulating valve"),
        choice(redeclare replaceable Buildings.Templates.Components.Valves.TwoWayModulating val
          "Two-way modulating valve")),
      Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=-90,
        origin={-40,-60})));

  // We allow for declaration but not through the parameter dialog box.
  replaceable Buildings.Fluid.HeatExchangers.WetCoilEffectivenessNTU hex(
    configuration=Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow,
    final use_Q_flow_nominal=true,
    final Q_flow_nominal=Q_flow_nominal,
    final T_a1_nominal=dat.TWatEnt_nominal,
    final T_a2_nominal=dat.TAirEnt_nominal,
    final w_a2_nominal=dat.wAirEnt_nominal,
    final dp1_nominal=if val.typ==Buildings.Templates.Components.Types.Valve.None
      then dpWat_nominal else 0,
    final dp2_nominal=dpAir_nominal)
  constrainedby Buildings.Fluid.Interfaces.PartialFourPortInterface(
    redeclare final package Medium1 = MediumChiWat,
    redeclare final package Medium2 = MediumAir,
    final m1_flow_nominal=mWat_flow_nominal,
    final m2_flow_nominal=mAir_flow_nominal)
    "Heat exchanger"
    annotation (
      Placement(transformation(extent={{10,4},{-10,-16}})));

  Buildings.Fluid.FixedResistances.Junction jun(
    redeclare final package Medium = MediumChiWat,
    final m_flow_nominal=mWat_flow_nominal*{1,-1,-1},
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    dp_nominal=fill(0, 3))
    if typVal==Buildings.Templates.Components.Types.Valve.ThreeWayModulating
    "Junction"
    annotation (
      Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={40,-60})));
  Buildings.Templates.BaseClasses.PassThroughFluid pas(
    redeclare final package Medium=MediumChiWat)
    if typVal<>Buildings.Templates.Components.Types.Valve.ThreeWayModulating
    "Direct pass through"
    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={60,-60})));
equation
  connect(port_a,hex. port_a2)
    annotation (Line(points={{-100,0},{-10,0}}, color={0,127,255}));
  connect(hex.port_b2, port_b)
    annotation (Line(points={{10,0},{100,0}}, color={0,127,255}));
  connect(val.port_a, hex.port_b1) annotation (Line(points={{-40,-50},{-40,-12},
          {-10,-12}}, color={0,127,255}));
  connect(val.port_b, port_bSou)
    annotation (Line(points={{-40,-70},{-40,-100}}, color={0,127,255}));
  connect(val.portByp_a, jun.port_3)
    annotation (Line(points={{-30,-60},{30,-60}}, color={0,127,255}));
  connect(jun.port_2, hex.port_a1)
    annotation (Line(points={{40,-50},{40,-12},{10,-12}}, color={0,127,255}));
  connect(port_aSou, jun.port_1)
    annotation (Line(points={{40,-100},{40,-70}}, color={0,127,255}));
  connect(port_aSou, pas.port_a) annotation (Line(points={{40,-100},{40,-80},{60,
          -80},{60,-70}}, color={0,127,255}));
  connect(pas.port_b, hex.port_a1)
    annotation (Line(points={{60,-50},{60,-12},{10,-12}}, color={0,127,255}));
  connect(val.bus, bus) annotation (Line(
      points={{-50,-60},{-60,-60},{-60,20},{0,20},{0,100}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(revisions="<html>
<p>
Using modified getReal function with annotation(__Dymola_translate=true)
avoids warning for non literal nominal attributes.
Not supported by OCT though:
Compliance error at line 8, column 4,
  Constructors for external objects is not supported in functions

</p>
</html>", info="<html>
<p>
This is a model for a chilled water coil with an optional
modulating valve.
</p>
</html>"));
end WaterBasedCooling;
