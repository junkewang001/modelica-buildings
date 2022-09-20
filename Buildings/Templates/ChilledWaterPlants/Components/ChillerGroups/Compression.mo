within Buildings.Templates.ChilledWaterPlants.Components.ChillerGroups;
model Compression "Group of compression chillers"
  extends
    Buildings.Templates.ChilledWaterPlants.Components.Interfaces.PartialChillerGroup;

  Buildings.Templates.Components.Chillers.Compression chi[nChi](
    redeclare each final package MediumChiWat=MediumChiWat,
    redeclare each final package MediumCon=MediumCon,
    each final allowFlowReversal=allowFlowReversal,
    each final typ=typChi,
    final dat=dat.datChi,
    each final energyDynamics=energyDynamics,
    each final tau=tau)
    "Chiller"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, rotation=90)));

  // Sensors
  // FIXME: bind have_sen to plant configuration per G36 rules.
  Buildings.Templates.Components.Sensors.Temperature TChiWatChiSup[nChi](
    redeclare each final package Medium=MediumChiWat,
    final m_flow_nominal=mChiWatChi_flow_nominal,
    each final have_sen=true,
    each final typ=Buildings.Templates.Components.Types.SensorTemperature.InWell)
    "Chiller CHW supply temperature"
    annotation (Placement(transformation(extent={{90,110},{110,130}})));
  Buildings.Templates.Components.Sensors.Temperature TChiWatChiRet[nChi](
    redeclare each final package Medium=MediumChiWat,
    final m_flow_nominal=mChiWatChi_flow_nominal,
    each final have_sen=true,
    each final typ=Buildings.Templates.Components.Types.SensorTemperature.InWell)
    "Chiller CHW return temperature"
    annotation (Placement(transformation(extent={{110,-110},{90,-90}})));
  Buildings.Templates.Components.Sensors.Temperature TConWatChiSup[nChi](
    redeclare each final package Medium=MediumCon,
    final m_flow_nominal=mConFluChi_flow_nominal,
    each have_sen=true,
    each final typ=Buildings.Templates.Components.Types.SensorTemperature.InWell)
    "Chiller CW supply temperature"
    annotation (Placement(transformation(extent={{-110,-110},{-90,-90}})));
  Buildings.Templates.Components.Sensors.Temperature TConWatChiRet[nChi](
    redeclare each final package Medium=MediumCon,
    final m_flow_nominal=mConFluChi_flow_nominal,
    each have_sen=true,
    each final typ=Buildings.Templates.Components.Types.SensorTemperature.InWell)
    "Chiller CW return temperature"
    annotation (Placement(transformation(extent={{-90,110},{-110,130}})));

  // CW isolation valve
  Buildings.Templates.Components.Valves.TwoWayModulating valConWatChiIsoMod[nChi](
    redeclare each final package Medium=MediumCon,
    each final allowFlowReversal=allowFlowReversal,
    final dat=datValConWatChiIso)
    if typValConWatIso_internal==Buildings.Templates.Components.Types.Valve.TwoWayModulating
    "Chiller CW isolation valve - Modulating"
    annotation (Placement(transformation(extent={{-150,150},{-170,170}})));
  Buildings.Templates.Components.Valves.TwoWayTwoPosition valConWatChiIsoTwo[nChi](
    redeclare each final package Medium = MediumCon,
    each final allowFlowReversal=allowFlowReversal,
    final dat=datValConWatChiIso,
    each final text_flip=true)
    if typValConWatIso_internal == Buildings.Templates.Components.Types.Valve.TwoWayTwoPosition
    "Chiller CW isolation valve - Two-position"
    annotation (Placement(transformation(extent={{-150,110},{-170,130}})));
  Buildings.Templates.Components.Routing.PassThroughFluid pasConWatChi[nChi](
    redeclare each final package Medium=MediumCon)
    if typValConWatIso_internal==Buildings.Templates.Components.Types.Valve.None
    "No chiller CW isolation valve"
    annotation (Placement(transformation(extent={{-150,90},{-170,70}})));

  // CHW isolation valve
  Buildings.Templates.Components.Valves.TwoWayTwoPosition valChiWatChiIsoTwo[nChi](
    redeclare each final package Medium = MediumChiWat,
    each final allowFlowReversal=allowFlowReversal,
    final dat=datValChiWatChiIso)
    if typValChiWatIso_internal == Buildings.Templates.Components.Types.Valve.TwoWayTwoPosition
    "Chiller CHW isolation valve - Two-position"
    annotation (Placement(transformation(extent={{150,110},{170,130}})));
  Buildings.Templates.Components.Routing.PassThroughFluid pasChiWatChi[nChi](
    redeclare each final package Medium=MediumChiWat)
    if typValChiWatIso_internal==Buildings.Templates.Components.Types.Valve.None
    "No chiller CHW isolation valve"
    annotation (Placement(transformation(extent={{150,90},{170,70}})));
  Buildings.Templates.Components.Valves.TwoWayModulating valChiWatChiIsoMod[nChi](
    redeclare each final package Medium = MediumChiWat,
    each final allowFlowReversal=allowFlowReversal,
    final dat=datValChiWatChiIso)
    if typValChiWatIso_internal == Buildings.Templates.Components.Types.Valve.TwoWayModulating
    "Chiller CHW isolation valve - Modulating"
    annotation (Placement(transformation(extent={{150,150},{170,170}})));
equation
  /* Control point connection - start */
  connect(bus.chi, chi.bus);
  connect(bus.TChiWatChiSup, TChiWatChiSup.y);
  connect(bus.TChiWatChiRet, TChiWatChiRet.y);
  connect(bus.TConWatChiSup, TConWatChiSup.y);
  connect(bus.TConWatChiRet, TConWatChiRet.y);
  connect(bus.valConWatChiIso, valConWatChiIsoMod.bus);
  connect(bus.valConWatChiIso, valConWatChiIsoTwo.bus);
  connect(bus.valChiWatChiIso, valChiWatChiIsoTwo.bus);
  connect(bus.valChiWatChiIso, valChiWatChiIsoMod.bus);
  /* Control point connection - stop */

  connect(TChiWatChiSup.port_b, valChiWatChiIsoTwo.port_a)
    annotation (Line(points={{110,120},{150,120}}, color={0,127,255}));
  connect(TChiWatChiSup.port_b, pasChiWatChi.port_a) annotation (Line(points={{110,120},
          {140,120},{140,80},{150,80}},      color={0,127,255}));
  connect(pasChiWatChi.port_b, ports_bChiWat) annotation (Line(points={{170,80},
          {180,80},{180,120},{200,120}}, color={0,127,255}));
  connect(valChiWatChiIsoTwo.port_b, ports_bChiWat)
    annotation (Line(points={{170,120},{200,120}}, color={0,127,255}));
  connect(valConWatChiIsoTwo.port_b, ports_bCon)
    annotation (Line(points={{-170,120},{-200,120}}, color={0,127,255}));
  connect(valConWatChiIsoMod.port_b, ports_bCon) annotation (Line(points={{-170,
          160},{-180,160},{-180,120},{-200,120}}, color={0,127,255}));
  connect(pasConWatChi.port_b, ports_bCon) annotation (Line(points={{-170,80},{-180,
          80},{-180,120},{-200,120}}, color={0,127,255}));
  connect(valConWatChiIsoMod.port_a, TConWatChiRet.port_b) annotation (Line(
        points={{-150,160},{-140,160},{-140,120},{-110,120}}, color={0,127,255}));
  connect(TConWatChiRet.port_b, valConWatChiIsoTwo.port_a)
    annotation (Line(points={{-110,120},{-150,120}}, color={0,127,255}));
  connect(TConWatChiRet.port_b, pasConWatChi.port_a) annotation (Line(points={{-110,
          120},{-140,120},{-140,80},{-150,80}}, color={0,127,255}));
  connect(chi.port_b1, TConWatChiRet.port_a) annotation (Line(points={{-6,-10},{
          -40,-10},{-40,120},{-90,120}}, color={0,127,255}));
  connect(ports_aChiWat, TChiWatChiRet.port_a)
    annotation (Line(points={{200,-100},{110,-100}}, color={0,127,255}));
  connect(TChiWatChiRet.port_b, chi.port_a2) annotation (Line(points={{90,-100},
          {20,-100},{20,-10},{6,-10}}, color={0,127,255}));
  connect(TConWatChiSup.port_b, chi.port_a1) annotation (Line(points={{-90,-100},
          {-20,-100},{-20,10},{-6,10}}, color={0,127,255}));
  connect(ports_aCon, TConWatChiSup.port_a)
    annotation (Line(points={{-200,-100},{-110,-100}}, color={0,127,255}));
  connect(chi.port_b2, TChiWatChiSup.port_a) annotation (Line(points={{6,10},{20,
          10},{20,120},{90,120}}, color={0,127,255}));
  connect(TChiWatChiSup.port_b, valChiWatChiIsoMod.port_a) annotation (Line(
        points={{110,120},{140,120},{140,160},{150,160}}, color={0,127,255}));
  connect(valChiWatChiIsoMod.port_b, ports_bChiWat) annotation (Line(points={{170,160},
          {180,160},{180,120},{200,120}},      color={0,127,255}));
  annotation(defaultComponentName="chi", Documentation(info="<html>
<p>
Current limitations:
</p>
<ul>
<li>
Same type of cooling fluid (air or water) for all chillers.
This is definitive considering the use of multiple-ports connectors
that require a unique medium model. 
</li> 
<li> 
Same type of CW (and CHW) isolation valve for all chillers.
This is a technical debt that will be purged when actuator models are 
refactored as container classes.
</li> 
Hence, only the same type of head pressure control for all chillers is supported.
(as the latter conditions the former).
</ul> 
</html>"));
end Compression;
