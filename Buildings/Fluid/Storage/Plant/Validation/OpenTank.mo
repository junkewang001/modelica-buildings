within Buildings.Fluid.Storage.Plant.Validation;
model OpenTank "(Draft)"
  extends Modelica.Icons.Example;
  extends Buildings.Fluid.Storage.Plant.Validation.BaseClasses.PartialOpenTank(
      souChi(final use_m_flow_in=true));

  Modelica.Blocks.Sources.TimeTable set_mTan_flow(table=[0,0; 3600/7,0; 3600/7,
        -1; 3600/7*3,-1; 3600/7*3,0; 3600/7*4,0; 3600/7*4,1; 3600/7*6,1; 3600/7
        *6,-1])
            "Tank flow rate setpoint"
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  Modelica.Blocks.Sources.BooleanTable uRemCha(table={0,3600/7*6}, startValue=
        true) "Tank is being charged remotely"
    annotation (Placement(transformation(extent={{100,80},{80,100}})));
  Modelica.Blocks.Sources.BooleanTable uOnl(table={3600/7*2})
    "True = plant online (outputting CHW to the network); False = offline"
    annotation (Placement(transformation(extent={{100,40},{80,60}})));
  Modelica.Blocks.Sources.TimeTable set_mChi_flow(table=[0,0; 3600/7,0; 3600/7,
        1; 3600/7*5,1; 3600/7*5,0]) "Chiller flow rate setpoint"
    annotation (Placement(transformation(extent={{-100,20},{-80,40}})));
  Buildings.Fluid.Storage.Plant.BaseClasses.ReversiblePumpValveControl
    conPumSec(tankIsOpen=true)
              "Control block for the secondary pump and near-by valves"
    annotation (Placement(transformation(extent={{10,40},{30,60}})));
equation
  connect(set_mChi_flow.y, souChi.m_flow_in)
    annotation (Line(points={{-79,30},{-70,30},{-70,38},{-62,38}},
                                                       color={0,0,127}));
  connect(set_mTan_flow.y, conPumSec.mTanSet_flow)
    annotation (Line(points={{-79,70},{6,70},{6,58},{9,58}}, color={0,0,127}));
  connect(conPumSec.uOnl, uOnl.y) annotation (Line(points={{32,54},{74,54},{74,50},
          {79,50}},     color={255,0,255}));
  connect(conPumSec.uRemCha, uRemCha.y)
    annotation (Line(points={{32,58},{32,90},{79,90}}, color={255,0,255}));
  connect(tanBra.mTan_flow, conPumSec.mTan_flow)
    annotation (Line(points={{-12,11},{-12,54},{9,54}}, color={0,0,127}));
  connect(supPum.yValDis_actual, conPumSec.yValDis_actual) annotation (Line(
        points={{12,11},{12,24},{2,24},{2,50},{9,50}}, color={0,0,127}));
  connect(supPum.yValCha_actual, conPumSec.yValCha_actual) annotation (Line(
        points={{16,11},{16,34},{6,34},{6,46},{9,46}}, color={0,0,127}));
  connect(conPumSec.yPum, supPum.yPum)
    annotation (Line(points={{20,39},{20,11}}, color={0,0,127}));
  connect(conPumSec.yValCha, supPum.yValCha)
    annotation (Line(points={{24,39},{24,11}}, color={0,0,127}));
  connect(conPumSec.yValDis, supPum.yValDis)
    annotation (Line(points={{28,39},{28,11}}, color={0,0,127}));
  annotation (
  experiment(Tolerance=1e-06, StopTime=3600),
    Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
    Icon(coordinateSystem(extent={{-100,-100},{100,100}})),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Storage/Plant/Validation/OpenTank.mos"
        "Simulate and plot"),
    Documentation(info="<html>
<p>
Documentation pending.
</p>
</html>", revisions="<html>
<ul>
<li>
April 11, 2022 by Hongxiang Fu:<br/>
First implementation. This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2859\">#2859</a>.
</li>
</ul>
</html>"));
end OpenTank;
