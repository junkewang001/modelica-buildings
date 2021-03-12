within Buildings.Templates.AHUs;
model VAVSingleDuct "VAV single duct with relief"
  extends Interfaces.Main(
    final typ=Types.Main.SupplyReturn,
    final typSup=Types.Supply.SingleDuct,
    final typRet=Types.Return.WithRelief);

  replaceable package MediumCoo=Buildings.Media.Water
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Cooling medium (such as CHW)"
    annotation(Dialog(enable=have_souCoiCoo));
  replaceable package MediumHea=Buildings.Media.Water
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Heating medium (such as HHW)"
    annotation(Dialog(enable=have_souCoiHea or have_souCoiReh));

  final inner parameter Types.Coil typCoiCoo = coiCoo.typ
    "Type of cooling coil"
    annotation (Evaluate=true,
      Dialog(group="Cooling coil"));
  final inner parameter Types.HeatExchanger typHexCoiCoo = coiCoo.typHex
    "Type of cooling coil heat exchanger"
    annotation (Evaluate=true,
      Dialog(group="Cooling coil"));
  final inner parameter Types.Actuator typActCoiCoo = coiCoo.typAct
    "Type of cooling coil actuator"
    annotation (Evaluate=true,
      Dialog(group="Cooling coil"));
  final inner parameter Boolean have_souCoiCoo = coiCoo.have_sou
    "Set to true for fluid ports on the source side"
    annotation (Evaluate=true, Dialog(group="Cooling coil"));

  final inner parameter Types.Coil typCoiHea=if coiHea.typ <> Types.Coil.None
       then coiHea.typ elseif coiReh.typ <> Types.Coil.None then coiReh.typ
       else Types.Coil.None
    "Type of heating coil"
    annotation (Evaluate=true, Dialog(group="Cooling coil"));
  final inner parameter Types.HeatExchanger typHexCoiHea=if coiHea.typ <>
      Types.Coil.None then coiHea.typHex elseif coiHea.typ <> Types.Coil.None
       then coiHea.typHex elseif coiReh.typ <> Types.Coil.None then coiReh.typHex
       else Types.HeatExchanger.None "Type of heating coil heat exchanger"
    annotation (Evaluate=true, Dialog(group="Cooling coil"));
  final inner parameter Types.Actuator typActCoiHea=if coiHea.typ <> Types.Coil.None
       then coiHea.typAct elseif coiReh.typ <> Types.Coil.None then coiReh.typAct
       else Types.Actuator.None
    "Type of heating coil actuator"
    annotation (Evaluate=true, Dialog(group="Cooling coil"));
  final parameter Boolean have_souCoiHea = coiHea.have_sou
    "Set to true for fluid ports on the source side"
    annotation (Evaluate=true, Dialog(group="Heating coil"));
  final parameter Boolean have_souCoiReh=coiReh.have_sou
    "Set to true for fluid ports on the source side"
    annotation (Evaluate=true, Dialog(group="Reheat coil"));

  final inner parameter Types.Fan typFanSup=
    if fanSupDra.typ<>Types.Fan.None then fanSupDra.typ
    elseif fanSupBlo.typ<>Types.Fan.None then fanSupBlo.typ
    else Types.Fan.None
    "Type of supply fan"
    annotation (
      Evaluate=true,
      Dialog(group="Supply fan"));

  Modelica.Fluid.Interfaces.FluidPort_b port_coiCooRet(
    redeclare package Medium = MediumCoo) if have_souCoiCoo
    "Cooling coil return port"
    annotation (Placement(
      transformation(extent={{30,-290},{50,-270}}),
      iconTransformation(extent={{50,-208},{70,-188}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_coiCooSup(
    redeclare package Medium = MediumCoo) if have_souCoiCoo
    "Cooling coil supply port"
    annotation (Placement(
        transformation(extent={{10,-290},{30,-270}}),   iconTransformation(
          extent={{10,-208},{30,-188}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_coiHeaRet(redeclare package Medium =
        MediumHea) if                        have_souCoiHea
    "Heating coil return port" annotation (Placement(transformation(extent={{-30,
            -290},{-10,-270}}), iconTransformation(extent={{-40,-208},{-20,-188}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_coiHeaSup(redeclare package Medium =
        MediumHea) if                        have_souCoiHea
    "Heating coil supply port" annotation (Placement(transformation(extent={{-50,
            -290},{-30,-270}}), iconTransformation(extent={{-80,-208},{-60,-188}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_coiRehRet(redeclare package Medium =
        MediumHea) if                        have_souCoiReh
    "Reheat coil return port" annotation (Placement(transformation(extent={{90,-290},
            {110,-270}}), iconTransformation(extent={{140,-208},{160,-188}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_coiRehSup(redeclare package Medium =
        MediumHea) if                        have_souCoiReh
    "Reheat coil supply port" annotation (Placement(transformation(extent={{70,-290},
            {90,-270}}), iconTransformation(extent={{100,-208},{120,-188}})));

  BoundaryConditions.WeatherData.Bus weaBus
    "Weather bus"
    annotation (Placement(transformation(extent={{-20,260},{20,300}}),
      iconTransformation(extent={{-20,182},{20,218}})));


  inner replaceable BaseClasses.Dampers.NoPath damRet
    constrainedby Interfaces.Damper(
      redeclare final package Medium=MediumAir)
    "Return air damper"
    annotation (
      choices(
        choice(redeclare BaseClasses.Dampers.NoPath damRet "No fluid path"),
        choice(redeclare BaseClasses.Dampers.Modulated damRet "Modulated damper")),
      Dialog(group="Economizer"),
      Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-120,-140})));
  inner replaceable BaseClasses.Dampers.NoPath damOutMin
    constrainedby Interfaces.Damper(
      redeclare final package Medium=MediumAir)
    "Minimum outdoor air damper"
    annotation (
      choices(
        choice(redeclare BaseClasses.Dampers.NoPath damOutMin "No damper"),
        choice(redeclare BaseClasses.Dampers.Modulated damOutMin "Modulated damper"),
        choice(redeclare BaseClasses.Dampers.TwoPosition damOutMin "Two-position damper")),
      Dialog(group="Economizer"),
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-230,-140})));
  inner replaceable BaseClasses.Dampers.None damOut
    constrainedby Interfaces.Damper(
      redeclare final package Medium=MediumAir)
    "Outdoor air damper"
    annotation (
      choices(
        choice(redeclare BaseClasses.Dampers.None damOut "No damper"),
        choice(redeclare BaseClasses.Dampers.Modulated damOut "Modulated damper"),
        choice(redeclare BaseClasses.Dampers.TwoPosition damOut "Two-position damper")),
      Dialog(group="Economizer"),
      Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-230,-200})));
  inner replaceable BaseClasses.Dampers.None damRel
    constrainedby Interfaces.Damper(
      redeclare final package Medium=MediumAir)
    "Relief air damper"
    annotation (
      choices(
        choice(redeclare BaseClasses.Dampers.None damRel "No damper"),
        choice(redeclare BaseClasses.Dampers.Modulated damRel "Modulated damper"),
        choice(redeclare BaseClasses.Dampers.TwoPosition damRel "Two-position damper")),
      Dialog(group="Economizer"),
      Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-230,-80})));
  replaceable BaseClasses.Sensors.None TOut
    constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Outdoor air temperature sensor"
    annotation (
      choices(
        choice(redeclare BaseClasses.Sensors.None TOut "No sensor"),
        choice(redeclare BaseClasses.Sensors.Temperature TOut "Temperature sensor")),
    Dialog(group="Sensors"),
    Placement(transformation(extent={{-210,-210},{-190,-190}})));
  replaceable BaseClasses.Sensors.None TOut1
    constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Outdoor air temperature sensor"
    annotation (
    choices(
      choice(redeclare BaseClasses.Sensors.None TOut1 "No sensor"),
      choice(redeclare BaseClasses.Sensors.Temperature TOut1 "Temperature sensor")),
    Dialog(group="Sensors", enable=damOutMin.typ<>Types.Damper.NoPath),
    Placement(transformation(extent={{-210,-150},{-190,-130}})));
  replaceable BaseClasses.Sensors.None VOut
    constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Outdoor air volume flow rate sensor"
    annotation (
      choices(
        choice(redeclare BaseClasses.Sensors.None VOut "No sensor"),
        choice(redeclare BaseClasses.Sensors.VolumeFlowRate VOut "Volume flow rate sensor")),
    Dialog(group="Economizer"),
    Placement(transformation(extent={{-180,-210},{-160,-190}})));
  replaceable BaseClasses.Sensors.None VOut1
    constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Outdoor air volume flow rate sensor"
    annotation (
    choices(
      choice(redeclare BaseClasses.Sensors.None VOut1 "No sensor"),
      choice(redeclare BaseClasses.Sensors.VolumeFlowRate VOut1 "Volume flow rate sensor")),
    Dialog(group="Economizer", enable=damOutMin.typ<>Types.Damper.NoPath),
    Placement(transformation(extent={{-180,-150},{-160,-130}})));
  replaceable BaseClasses.Sensors.None dpOut
    constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Outdoor air damper differential pressure"
    annotation (
      choices(
        choice(redeclare BaseClasses.Sensors.None dpOut "No sensor"),
        choice(redeclare BaseClasses.Sensors.DifferentialPressure dpOut "Differential pressure sensor")),
      Dialog(group="Economizer", enable=damOutMin.typ<>Types.Damper.NoPath),
      Placement(transformation(extent={{-270,-150},{-250,-130}})));
  replaceable BaseClasses.Sensors.None TMix constrainedby Interfaces.Sensor(
      redeclare final package Medium=MediumAir)
    "Mixed air temperature sensor" annotation (
    choices(
      choice(redeclare BaseClasses.Sensors.None TMix "No sensor"),
      choice(redeclare BaseClasses.Sensors.Temperature TMix "Temperature sensor")),
      Dialog(group="Economizer"),
      Placement(transformation(extent={{-100,-210},{-80,-190}})));

  inner replaceable BaseClasses.Fans.None fanSupBlo
    constrainedby Interfaces.Fan(
      redeclare final package Medium=MediumAir)
    "Supply fan - Blow through"
    annotation (
    choicesAllMatching=true,
    Dialog(group="Supply fan", enable=fanSupDra==Types.Fan.None),
    Placement(transformation(extent={{-70,-210},{-50,-190}})));

  inner replaceable BaseClasses.Coils.None coiHea constrainedby Interfaces.Coil(
      redeclare final package MediumAir = MediumAir, redeclare final package
      MediumSou = MediumHea) "Heating coil" annotation (
    choicesAllMatching=true,
    Dialog(group="Heating coil"),
    Placement(transformation(extent={{-40,-210},{-20,-190}})));
  replaceable BaseClasses.Sensors.None THea constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Heating coil"
    annotation (
      choices(
        choice(redeclare BaseClasses.Sensors.None THea "No sensor"),
        choice(redeclare BaseClasses.Sensors.Temperature THea "Temperature sensor")),
    Dialog(group="Heating coil", enable=coiHea<>Types.Coil.None),
    Placement(transformation(extent={{-10,-210},{10,-190}})));

  inner replaceable BaseClasses.Coils.None coiCoo
    constrainedby Interfaces.Coil(
      redeclare final package MediumAir = MediumAir,
      redeclare final package MediumSou = MediumCoo)
    "Cooling coil"
    annotation (
      choicesAllMatching=true,
      Dialog(group="Cooling coil"),
      Placement(transformation(extent={{20,-210},{40,-190}})));
  replaceable BaseClasses.Sensors.None TCoo
    constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Cooling coil leaving air temperature sensor"
    annotation (
      choices(
        choice(redeclare BaseClasses.Sensors.None TCoo "No sensor"),
        choice(redeclare BaseClasses.Sensors.Temperature TCoo "Temperature sensor")),
      Dialog(group="Cooling coil", enable=coiCoo<>Types.Coil.None),
      Placement(transformation(extent={{50,-210},{70,-190}})));

  inner replaceable BaseClasses.Coils.None coiReh
    constrainedby Interfaces.Coil(
      redeclare final package MediumAir = MediumAir,
      redeclare final package MediumSou = MediumHea)
    "Reheat coil"
    annotation (
      choicesAllMatching=true,
      Dialog(group="Reheat coil"),
      Placement(transformation(extent={{80,-210},{100,-190}})));

  inner replaceable BaseClasses.Fans.None fanSupDra
    constrainedby Interfaces.Fan(
      redeclare final package Medium=MediumAir)
    "Supply fan - Draw through"
    annotation (
      choicesAllMatching=true,
      Dialog(group="Supply fan", enable=fanSupBlo==Types.Fan.None),
      Placement(transformation(extent={{110,-210},{130,-190}})));
  replaceable BaseClasses.Sensors.None VSup
    constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Supply air volume flow rate sensor"
    annotation (
      choices(
        choice(redeclare BaseClasses.Sensors.None VSup "No sensor"),
        choice(redeclare BaseClasses.Sensors.VolumeFlowRate VSup "Volume flow rate sensor")),
      Dialog(group="Supply fan"),
      Placement(transformation(extent={{142,-210},{162,-190}})));

  inner replaceable BaseClasses.Controls.Dummy conAhu
    constrainedby Interfaces.Controller
    "AHU controller"
    annotation (
      choicesAllMatching=true,
      Dialog(group="Controller"),
      Placement(transformation(extent={{-60,90},{-40,110}})));

  // FIXME: Dummy default values fo testing purposes only.
  Fluid.FixedResistances.PressureDrop resRet(
    redeclare final package Medium = MediumAir,
    m_flow_nominal=1,
    dp_nominal=100)
    annotation (Placement(transformation(extent={{190,-90},{170,-70}})));
  Fluid.FixedResistances.PressureDrop resSup(
    redeclare final package Medium = MediumAir,
    m_flow_nominal=1,
    dp_nominal=100)
    annotation (Placement(transformation(extent={{172,-210},{192,-190}})));

  replaceable BaseClasses.Sensors.None TSup constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Supply air temperature sensor"
    annotation (
    choices(choice(redeclare BaseClasses.Sensors.None TSup "No sensor"),
      choice(redeclare
          BaseClasses.Sensors.Temperature TSup "Temperature sensor")),
      Dialog(group="Sensors"),
      Placement(transformation(extent={{200,-210},{220,-190}})));
  replaceable BaseClasses.Sensors.None xSup constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Supply air humidity ratio sensor"
    annotation (
      choices(choice(redeclare BaseClasses.Sensors.None xSup "No sensor"),
        choice(redeclare
          BaseClasses.Sensors.HumidityRatio xSup "Humidity ratio sensor")),
      Dialog(group="Sensors"),
      Placement(transformation(extent={{230,-210},{250,-190}})));
  replaceable BaseClasses.Sensors.None pSup_rel
    constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Duct static pressure sensor"
    annotation (
      choices(
        choice(redeclare BaseClasses.Sensors.None pSup_rel "No sensor"),
        choice(redeclare BaseClasses.Sensors.DifferentialPressure pSup_rel "Differential pressure sensor")),
      Dialog(group="Sensors"),
      Placement(transformation(extent={{260,-210},{280,-190}})));

  inner replaceable BaseClasses.Fans.None fanRet
    constrainedby Interfaces.Fan(
      redeclare final package Medium=MediumAir)
    "Return fan"
    annotation (
      choicesAllMatching=true,
      Dialog(group="Return/relief fan", enable=fanRel.typ==BaseClasses.Fans.None),
      Placement(transformation(extent={{20,-90},{0,-70}})));

  inner replaceable BaseClasses.Fans.None fanRel
    constrainedby Interfaces.Fan(
      redeclare final package Medium=MediumAir)
    "Relief fan"
    annotation (
      choicesAllMatching=true,
      Dialog(group="Return/relief fan", enable=fanRet.typ==BaseClasses.Fans.None),
      Placement(transformation(extent={{-140,-90},{-160,-70}})));
  replaceable BaseClasses.Sensors.None VRet
    constrainedby Interfaces.Sensor(
      redeclare final package Medium=MediumAir)
    "Return air volume flow rate sensor"
    annotation (
      choices(
        choice(redeclare BaseClasses.Sensors.None VRet "No sensor"),
        choice(redeclare BaseClasses.Sensors.VolumeFlowRate VRet "Volume flow rate sensor")),
      Dialog(group="Return/relief fan"),
      Placement(transformation(extent={{-10,-90},{-30,-70}})));
  Fluid.Sensors.RelativePressure pInd_rel(
    redeclare final package Medium=MediumAir)
    "Building static pressure"
    annotation (Placement(transformation(extent={{30,230},{10,250}})));
  replaceable BaseClasses.Sensors.None pRet_rel
    constrainedby Interfaces.Sensor(
      redeclare final package Medium = MediumAir)
    "Return static pressure sensor"
    annotation (
      choices(
        choice(redeclare BaseClasses.Sensors.None pRet_rel "No sensor"),
        choice(redeclare BaseClasses.Sensors.DifferentialPressure pRet_rel "Differential pressure sensor")),
      Dialog(group="Return/relief fan"),
      Placement(transformation(extent={{-60,-90},{-80,-70}})));

  Fluid.Sources.Outside out(
    redeclare final package Medium=MediumAir,
    final nPorts=1)
    "Outdoor conditions"
    annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-40,250})));
  Fluid.Sources.Boundary_pT ind(
    redeclare final package Medium=MediumAir,
    final use_p_in=true,
    final nPorts=
      1 +
      (if pSup_rel.typ==Types.Sensor.DifferentialPressure then 1 else 0) +
      (if pRet_rel.typ==Types.Sensor.DifferentialPressure then 1 else 0))
    "Indoor pressure"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={40,250})));

equation
  connect(port_coiCooSup, coiCoo.port_aSou) annotation (Line(points={{20,-280},{
          20,-220},{26,-220},{26,-210}},   color={0,127,255}));
  connect(coiCoo.port_bSou, port_coiCooRet) annotation (Line(points={{34,-210},{
          34,-220},{40,-220},{40,-280}},
                                      color={0,127,255}));

  connect(resRet.port_a, port_Ret)
    annotation (Line(points={{190,-80},{300,-80}}, color={0,127,255}));
  connect(weaBus, coiCoo.weaBus) annotation (Line(
      points={{0,280},{0,80},{24,80},{24,-190}},
      color={255,204,51},
      thickness=0.5));
  connect(ahuBus, coiCoo.ahuBus) annotation (Line(
      points={{-300,0},{30,0},{30,-190}},
      color={255,204,51},
      thickness=0.5));
  connect(ahuBus, fanSupBlo.ahuBus) annotation (Line(
      points={{-300,0},{-60,0},{-60,-190}},
      color={255,204,51},
      thickness=0.5));
  connect(ahuBus, fanSupDra.ahuBus) annotation (Line(
      points={{-300,0},{120,0},{120,-190}},
      color={255,204,51},
      thickness=0.5));
  connect(conAhu.ahuBus, ahuBus) annotation (Line(
      points={{-60,100},{-280,100},{-280,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(conAhu.terBus, terBus) annotation (Line(
      points={{-40,100},{280,100},{280,0},{300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(TMix.port_b, fanSupBlo.port_a)
    annotation (Line(points={{-80,-200},{-70,-200}},   color={0,127,255}));
  connect(THea.port_b, coiCoo.port_a)
    annotation (Line(points={{10,-200},{20,-200}}, color={0,127,255}));
  connect(coiCoo.port_b, TCoo.port_a)
    annotation (Line(points={{40,-200},{50,-200}}, color={0,127,255}));
  connect(TCoo.port_b, coiReh.port_a)
    annotation (Line(points={{70,-200},{80,-200}}, color={0,127,255}));
  connect(resSup.port_b, TSup.port_a)
    annotation (Line(points={{192,-200},{200,-200}}, color={0,127,255}));
  connect(TSup.port_b, xSup.port_a)
    annotation (Line(points={{220,-200},{230,-200}}, color={0,127,255}));
  connect(xSup.port_b, pSup_rel.port_a)
    annotation (Line(points={{250,-200},{260,-200}}, color={0,127,255}));
  connect(pSup_rel.port_b, port_Sup)
    annotation (Line(points={{280,-200},{300,-200}}, color={0,127,255}));
  connect(port_coiHeaSup, coiHea.port_aSou) annotation (Line(points={{-40,-280},
          {-40,-220},{-34,-220},{-34,-210}}, color={0,127,255}));
  connect(coiHea.port_bSou, port_coiHeaRet) annotation (Line(points={{-26,-210},
          {-26,-220},{-20,-220},{-20,-280}}, color={0,127,255}));
  connect(port_coiRehSup, coiReh.port_aSou) annotation (Line(points={{80,-280},{
          80,-220},{86,-220},{86,-210}}, color={0,127,255}));
  connect(coiReh.port_bSou, port_coiRehRet) annotation (Line(points={{94,-210},{
          94,-220},{100,-220},{100,-280}}, color={0,127,255}));
  connect(ahuBus, coiHea.ahuBus) annotation (Line(
      points={{-300,0},{-30,0},{-30,-190}},
      color={255,204,51},
      thickness=0.5));
  connect(ahuBus, coiReh.ahuBus) annotation (Line(
      points={{-300,0},{90,0},{90,-190}},
      color={255,204,51},
      thickness=0.5));
  connect(TMix.ahuBus, ahuBus) annotation (Line(
      points={{-90,-190},{-90,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(THea.ahuBus, ahuBus) annotation (Line(
      points={{0,-190},{0,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(TSup.ahuBus, ahuBus) annotation (Line(
      points={{210,-190},{210,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(xSup.ahuBus, ahuBus) annotation (Line(
      points={{240,-190},{240,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(pSup_rel.ahuBus, ahuBus) annotation (Line(
      points={{270,-190},{270,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(TCoo.ahuBus, ahuBus) annotation (Line(
      points={{60,-190},{60,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(fanRel.ahuBus, ahuBus) annotation (Line(
      points={{-150,-70},{-150,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(fanRet.ahuBus, ahuBus) annotation (Line(
      points={{10,-70},{10,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(damRel.port_b, port_Exh)
    annotation (Line(points={{-240,-80},{-300,-80}}, color={0,127,255}));
  connect(fanRel.port_b, damRel.port_a)
    annotation (Line(points={{-160,-80},{-220,-80}}, color={0,127,255}));
  connect(resRet.port_b, fanRet.port_a) annotation (Line(points={{170,-80},{20,-80}},
                               color={0,127,255}));
  connect(damRet.port_b, TMix.port_a) annotation (Line(points={{-120,-150},{-120,
          -200},{-100,-200}},      color={0,127,255}));
  connect(coiReh.port_b, fanSupDra.port_a)
    annotation (Line(points={{100,-200},{110,-200}}, color={0,127,255}));
  connect(damOutMin.port_b, TOut1.port_a)
    annotation (Line(points={{-220,-140},{-210,-140}}, color={0,127,255}));
  connect(TOut1.port_b, VOut1.port_a)
    annotation (Line(points={{-190,-140},{-180,-140}}, color={0,127,255}));
  connect(port_Out, dpOut.port_a) annotation (Line(points={{-300,-200},{-280,-200},
          {-280,-140},{-270,-140}}, color={0,127,255}));
  connect(dpOut.port_b, damOutMin.port_a)
    annotation (Line(points={{-250,-140},{-240,-140}}, color={0,127,255}));
  connect(VOut1.port_b, TMix.port_a) annotation (Line(points={{-160,-140},{-140,
          -140},{-140,-200},{-100,-200}}, color={0,127,255}));
  connect(dpOut.port_bRef, TMix.port_a) annotation (Line(points={{-260,-150},{-260,
          -160},{-140,-160},{-140,-200},{-100,-200}}, color={0,127,255}));
  connect(fanSupDra.port_b, VSup.port_a)
    annotation (Line(points={{130,-200},{142,-200}}, color={0,127,255}));
  connect(VSup.port_b, resSup.port_a)
    annotation (Line(points={{162,-200},{172,-200}}, color={0,127,255}));
  connect(damRel.ahuBus, ahuBus) annotation (Line(
      points={{-230,-70},{-230,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(VOut1.ahuBus, ahuBus) annotation (Line(
      points={{-170,-130},{-170,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(TOut1.ahuBus, ahuBus) annotation (Line(
      points={{-200,-130},{-200,-8},{-300,-8},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(damOutMin.ahuBus, ahuBus) annotation (Line(
      points={{-230,-130},{-230,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(dpOut.ahuBus, ahuBus) annotation (Line(
      points={{-260,-130},{-260,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(damOut.ahuBus, ahuBus) annotation (Line(
      points={{-230,-190},{-230,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(TOut.ahuBus, ahuBus) annotation (Line(
      points={{-200,-190},{-200,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(VOut.ahuBus, ahuBus) annotation (Line(
      points={{-170,-190},{-170,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(VSup.ahuBus, ahuBus) annotation (Line(
      points={{152,-190},{152,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(VRet.ahuBus, ahuBus) annotation (Line(
      points={{-20,-70},{-20,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(damRet.ahuBus, ahuBus) annotation (Line(
      points={{-110,-140},{-100,-140},{-100,0},{-300,0}},
      color={255,204,51},
      thickness=0.5));
  connect(fanRet.port_b, VRet.port_a)
    annotation (Line(points={{0,-80},{-10,-80}}, color={0,127,255}));
  connect(fanSupBlo.port_b, coiHea.port_a)
    annotation (Line(points={{-50,-200},{-40,-200}}, color={0,127,255}));
  connect(coiHea.port_b, THea.port_a)
    annotation (Line(points={{-20,-200},{-10,-200}}, color={0,127,255}));
  connect(weaBus, out.weaBus) annotation (Line(
      points={{0,280},{0,276},{-40,276},{-40,260},{-39.8,260}},
      color={255,204,51},
      thickness=0.5));
  connect(ahuBus.ahuI.pInd, ind.p_in) annotation (Line(
      points={{-300.1,0.1},{-300,0.1},{-300,0},{60,0},{60,268},{48,268},{48,262}},
      color={255,204,51},
      thickness=0.5));
  connect(damOut.port_b, TOut.port_a)
    annotation (Line(points={{-220,-200},{-210,-200}}, color={0,127,255}));
  connect(TOut.port_b, VOut.port_a)
    annotation (Line(points={{-190,-200},{-180,-200}}, color={0,127,255}));
  connect(port_Out, damOut.port_a)
    annotation (Line(points={{-300,-200},{-240,-200}}, color={0,127,255}));
  connect(VOut.port_b, TMix.port_a)
    annotation (Line(points={{-160,-200},{-100,-200}}, color={0,127,255}));
  connect(pInd_rel.port_b, out.ports[1])
    annotation (Line(points={{10,240},{-40,240}}, color={0,127,255}));
  connect(ind.ports[1], pInd_rel.port_a)
    annotation (Line(points={{40,240},{30,240}},      color={0,127,255}));
  connect(ind.ports[2], pSup_rel.port_bRef) annotation (Line(points={{40,240},{288,
          240},{288,-220},{270,-220},{270,-210}}, color={0,127,255}));
  connect(pRet_rel.port_b, fanRel.port_a)
    annotation (Line(points={{-80,-80},{-140,-80}}, color={0,127,255}));
  connect(pRet_rel.port_b, damRet.port_a) annotation (Line(points={{-80,-80},{-120,
          -80},{-120,-130}}, color={0,127,255}));
  connect(VRet.port_b, pRet_rel.port_a)
    annotation (Line(points={{-30,-80},{-60,-80}}, color={0,127,255}));
  connect(pRet_rel.ahuBus, ahuBus.ahuI.pRet_rel) annotation (Line(
      points={{-70,-70},{-70,0.1},{-300.1,0.1}},
      color={255,204,51},
      thickness=0.5));
  connect(pRet_rel.port_bRef, ind.ports[3]) annotation (Line(points={{-70,-90},{
          -70,-100},{40,-100},{40,240}},           color={0,127,255}));
  connect(pInd_rel.p_rel, ahuBus.ahuI.pRet_rel.ahuI.pInd_rel) annotation (Line(
        points={{20,231},{20,0.1},{-300.1,0.1}}, color={0,0,127}));
  annotation (
    defaultComponentName="ahu",
    Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
    coordinateSystem(preserveAspectRatio=false), graphics={Text(
          extent={{-86,230},{124,210}},
          lineColor={0,127,255},
          pattern=LinePattern.Dash,
          textString="No further connection allowed to those two boundary conditions")}));
end VAVSingleDuct;