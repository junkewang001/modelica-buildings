within Buildings.Templates.AHUs.Coils;
model WaterBased
  extends Interfaces.Coil(
    final typ=Types.Coil.WaterBased,
    final have_weaBus=false,
    final have_sou=true,
    final typAct=act.typ,
    final typHex=hex.typ);
  extends Data.WaterBased
    annotation (IconMap(primitivesVisible=false));

  replaceable Actuators.None act
    constrainedby Interfaces.Actuator(
      redeclare final package Medium = MediumSou)
    "Actuator"
    annotation (
      choicesAllMatching=true,
      Placement(transformation(extent={{-10,-70},{10,-50}})));

  // TODO: conditional choices based on funStr to restrict HX models for cooling.
  replaceable HeatExchangers.DryCoilEffectivenessNTU hex constrainedby
    Interfaces.HeatExchangerWater(
    redeclare final package Medium1 = MediumSou,
    redeclare final package Medium2 = MediumAir,
    final m1_flow_nominal=mWat_flow_nominal,
    final m2_flow_nominal=mAir_flow_nominal,
    final dp1_nominal=if typAct==Types.Actuator.None then dpWat_nominal else 0,
    final dp2_nominal=dpAir_nominal)
    "Heat exchanger"
    annotation (
      choicesAllMatching=true,
      Placement(transformation(extent={{10,4},{-10,-16}})));
equation
  /*
  Bug in Dymola: act.typ<>Types.Actuator.None is not necessary from Modelica spec.
  OCT can compile without that condition and automatically remove the connect 
  clause when the act.y is coniditionally removed.
  */
  if funStr=="Cooling" and act.typ<>Types.Actuator.None then
    connect(ahuBus.ahuO.yCoiCoo, act.y) annotation (Line(
        points={{0.1,100.1},{0.1,20},{-40,20},{-40,-60},{-11,-60}},
        color={255,204,51},
        thickness=0.5));
  elseif act.typ<>Types.Actuator.None then
    connect(ahuBus.ahuO.yCoiHea, act.y) annotation (Line(
        points={{0.1,100.1},{0.1,20},{-40,20},{-40,-60},{-11,-60}},
        color={255,204,51},
        thickness=0.5));
  end if;

  connect(port_aSou, act.port_aSup) annotation (Line(points={{-40,-100},{-40,-80},
          {-4,-80},{-4,-70}}, color={0,127,255}));
  connect(act.port_bRet, port_bSou) annotation (Line(points={{4,-70},{4,-80},{40,
          -80},{40,-100}}, color={0,127,255}));
  connect(act.port_bSup,hex. port_a1) annotation (Line(points={{-4,-50},{-4,-22},
          {20,-22},{20,-12},{10,-12}}, color={0,127,255}));
  connect(hex.port_b1, act.port_aRet) annotation (Line(points={{-10,-12},{-20,-12},
          {-20,-24},{4,-24},{4,-50}}, color={0,127,255}));
  connect(port_a,hex. port_a2)
    annotation (Line(points={{-100,0},{-10,0}}, color={0,127,255}));
  connect(hex.port_b2, port_b) annotation (Line(points={{10,0},{56,0},{56,0},{100,
          0}}, color={0,127,255}));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end WaterBased;
