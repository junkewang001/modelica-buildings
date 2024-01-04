within Buildings.Experimental.DHC.EnergyTransferStations.Combined;
model HeatPumpHeatExchangerDHWTank
  "Model of a substation with heat pump for heating, heat pump with storage tank for domestic hot water, and compressor-less cooling"
  extends
    Buildings.Experimental.DHC.EnergyTransferStations.Combined.BaseClasses.PartialHeatPumpHeatExchanger(
    volMix_b(nPorts=4),
    volMix_a(nPorts=4));
  Buildings.Fluid.Sources.Boundary_pT souDCW(
    redeclare final package Medium = MediumBui,
    use_T_in=true,
    nPorts=1)  if have_hotWat "Source for domestic cold water"
                                 annotation (Placement(
      transformation(
      extent={{10,-10},{-10,10}},
      rotation=180,
      origin={-52,-56})));
  Buildings.Experimental.DHC.EnergyTransferStations.Combined.Subsystems.HeatPumpDHWTank proHotWat(
    redeclare final package Medium1 = MediumBui,
    redeclare final package Medium2 = MediumSer,
    final COP_nominal=COPHotWat_nominal,
    TCon_nominal=THotWatSup_nominal,
    TEva_nominal=TDisWatMin - dT_nominal,
    QHotWat_flow_nominal=QHotWat_flow_nominal,
    dT_nominal=dT_nominal,
    final allowFlowReversal1=allowFlowReversalBui,
    final allowFlowReversal2=allowFlowReversalSer,
    dp1_nominal=6000,
    dp2_nominal=6000,
    datWatHea=datWatHea) if have_hotWat
    "Subsystem for hot water production"
    annotation (Placement(transformation(extent={{30,24},{50,44}})));
  parameter Buildings.Experimental.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
    datWatHea "Performance data"
    annotation (Placement(transformation(extent={{36,48},{48,60}})));
  Buildings.Experimental.DHC.Loads.HotWater.ThermostaticMixingValve theMixVal(
    redeclare package Medium = MediumBui,
    mMix_flow_nominal=QHotWat_flow_nominal/cpBui_default/(THotWatSup_nominal - TColWat_nominal))
     if have_hotWat
     "Thermostatic mixing valve"
    annotation (Placement(transformation(extent={{-20,50},{-40,72}})));
  Buildings.Experimental.DHC.EnergyTransferStations.BaseClasses.Junction dcwSpl(
      redeclare final package Medium = MediumBui, final m_flow_nominal=
        datWatHea.mDom_flow_nominal*{1,-1,-1})
                                             "Splitter for domestic cold water"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-10,0})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai(
    k=1/QHotWat_flow_nominal) if have_hotWat
    annotation (Placement(transformation(extent={{-80,50},{-60,70}})));
equation

  connect(proHotWat.port_b2, volMix_b.ports[4])
    annotation (Line(points={{30,28},{28,28},{28,0},{260,0},{260,-360}},
                                                           color={0,127,255}));
  connect(proHotWat.PHea, PHeaTot.u[2]) annotation (Line(points={{52,37},{66,37},
          {66,38},{268,38},{268,80.5}},
        color={0,0,127}));
  connect(proHotWat.PPum, PPumHeaTot.u[2]) annotation (Line(points={{52,34},{
          176,34},{176,420.5},{188,420.5}},
                                    color={0,0,127}));
  connect(proHotWat.mEva_flow, masFloHeaTot.u[2]) annotation (Line(points={{52,31},
          {68,31},{68,30},{256,30},{256,-139.5},{268,-139.5}},
                                                           color={0,0,127}));
  connect(proHotWat.mEva_flow, masFloHea.u2) annotation (Line(points={{52,31},{
          58,31},{58,-242},{-6,-242},{-6,-252}},
                                              color={0,0,127}));
  connect(proHotWat.port_a2, volMix_a.ports[4]) annotation (Line(points={{50,28},
          {54,28},{54,20},{-260,20},{-260,-360}},             color={0,127,255}));
  connect(souDCW.ports[1], dcwSpl.port_1) annotation (Line(points={{-42,-56},{
          -10,-56},{-10,-10}},color={0,127,255}));
  connect(dcwSpl.port_3, proHotWat.port_a1)
    annotation (Line(points={{0,-6.66134e-16},{10,-6.66134e-16},{10,40},{30,40}},
                                                           color={0,127,255}));
  connect(dcwSpl.port_2, theMixVal.port_col) annotation (Line(points={{-10,10},
          {-10,52.2},{-20,52.2}}, color={0,127,255}));
  connect(proHotWat.port_b1, theMixVal.port_hot) annotation (Line(points={{50,40},
          {60,40},{60,80},{0,80},{0,56.6},{-20,56.6}},
                                           color={0,127,255}));
  connect(souDCW.T_in, TColWat) annotation (Line(points={{-64,-60},{-156,-60},{
          -156,-80},{-320,-80}}, color={0,0,127}));
  connect(THotWatSupSet, theMixVal.TMixSet) annotation (Line(points={{-320,-40},
          {-32,-40},{-32,32},{-6,32},{-6,64},{-14,64},{-14,63.2},{-19,63.2}},
                                                            color={0,0,127}));
  connect(QReqHotWat_flow, gai.u) annotation (Line(points={{-320,-120},{-288,-120},
          {-288,60},{-82,60}},
                         color={0,0,127}));
  connect(gai.y, theMixVal.yMixSet) annotation (Line(points={{-58,60},{-48,60},
          {-48,78},{-6,78},{-6,70},{-12,70},{-12,69.8},{-19,69.8}},
                                                  color={0,0,127}));
  connect(enaSHW.y, proHotWat.uEna) annotation (Line(points={{-118,80},{-108,80},
          {-108,43},{28,43}}, color={255,0,255}));
  annotation (
  Documentation(info="<html>
<p>
This model uses the base energy transfer station defined in
<a href=\"modelica://Buildings.Experimental.DHC.EnergyTransferStations.Combined.BaseClasses.PartialHeatPumpHeatExchanger\">
Buildings.Experimental.DHC.EnergyTransferStations.Combined.BaseClasses.PartialHeatPumpHeatExchanger</a>.
</p>
<h4>Domestic Hot Water</h4>
<p>
Domestic hot water is produced using a dedicated water-to-water heat pump
with storage tank.
</p>
<p>
Heating is enabled based on the input signal <code>uSHW</code>
which is held for <i>15</i> minutes, meaning that,
when enabled, the mode remains active for at least <i>15</i> minutes and,
when disabled, the mode cannot be enabled again for at least <i>15</i> minutes.
The enable signal should be computed externally based
on a schedule (to lock out the system during off-hours), ideally in conjunction
with the number of requests or any other signal representative of the load.
</p>
<p>
When enabled,
</p>
<ul>
<li>
The heat pump and the evaporator and condenser hydronics are controlled
based on the principles described in
<a href=\"modelica://Buildings.Experimental.DHC.EnergyTransferStations.Combined.Subsystems.HeatPumpDHWTank\">
Buildings.Experimental.DHC.EnergyTransferStations.Combined.Subsystems.HeatPumpDHWTank</a>.
</li>
<li>
The mass flow rate of water leaving the domestic hot water heat exchanger is computed based on the domestic hot water
heating load (input <code>QReqHotWat_flow</code>) combined with the operation of a thermostatic
mixing valve used to mix down the temperature of hot water leaving the domestic hot water heat exchanger
to the temperature distributed to fixtures (parameter <code>THotWatSup_nominal</code>)
using domestic cold water at the cold water temperature (input <code>TColWat</code>).
The desired water flow rate leaving the thermostatic mixing valve
is determined according to the following equation:
<p align=\"center\" style=\"font-style:italic;\">
<code>QReqHotWat_flow</code> = m&#775; cp (<code>THotWatSup_nominal</code> - <code>TColWat</code>)
</p>
</li>
</ul>
</html>",
  revisions="<html>
<ul>
<li>
September 13, 2023, by David Blum:<br/>
Extended from partial base class.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/3063\">
issue 3063</a>.
</li>
<li>
May 17, 2023, by David Blum:<br/>
Assigned dp_nominal to <code>pum1HexChi</code>.<br/>
Corrected calculation of heat pump evaporator mass flow control.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/3379\">
issue 3379</a>.
</li>
<li>
February 23, 2021, by Antoine Gautier:<br/>
Refactored with subsystem models and partial ETS base class.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1769\">
issue 1769</a>.
</li>
<li>
December 12, 2017, by Michael Wetter:<br/>
Removed call to <code>Modelica.Utilities.Files.loadResource</code>.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1097\">issue 1097</a>.
</li>
</ul>
</html>"), Icon(graphics={
        Rectangle(
          extent={{-70,36},{-68,26}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-1.5,5.5},{1.5,-5.5}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          origin={-13.5,26.5},
          rotation=90)}));
end HeatPumpHeatExchangerDHWTank;
