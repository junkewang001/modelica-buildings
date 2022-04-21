within Buildings.Fluid.Storage.Plant.Validation.BaseClasses;
partial model RemoteChargingSchedule
  "Schedules for validation models with remote charging"
  Modelica.Blocks.Sources.BooleanTable uOnl(table={3600/7*2})
    "True = plant online (outputting CHW to the network); False = offline"
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  Modelica.Blocks.Sources.BooleanTable uRemCha(table={0,3600/7*6}, startValue=true)
              "Tank is being charged remotely"
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  Modelica.Blocks.Sources.TimeTable set_mTan_flow(table=[0,0; 3600/7,0; 3600/7,-1;
        3600/7*3,-1; 3600/7*3,0; 3600/7*4,0; 3600/7*4,1; 3600/7*6,1; 3600/7*6,-1])
            "Tank flow rate setpoint"
    annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
  Modelica.Blocks.Sources.TimeTable set_mChi_flow(table=[0,0; 3600/7,0; 3600/7,
        1; 3600/7*2,1; 3600/7*2,2; 3600/7*3,2; 3600/7*3,1; 3600/7*5,1; 3600/7*5,
        0])                         "Chiller flow rate setpoint"
    annotation (Placement(transformation(extent={{-100,-40},{-80,-20}})));
  Buildings.Fluid.Storage.Plant.BaseClasses.PumpValveControl conPumVal
    "Control block for the secondary pump and near-by valves"
    annotation (Placement(transformation(extent={{10,40},{30,60}})));
equation
  connect(uRemCha.y, conPumVal.uRemCha) annotation (Line(points={{-39,70},{34,70},
          {34,60},{32,60}}, color={255,0,255}));
  connect(uOnl.y, conPumVal.uOnl) annotation (Line(points={{-79,90},{38,90},{38,
          56},{32,56}}, color={255,0,255}));
  connect(conPumVal.mTanSet_flow, set_mTan_flow.y) annotation (Line(points={{9,58},{
          -60,58},{-60,50},{-79,50}},  color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<p>
This partial model contains the following schedules for a storage plant
that allows remote charging.
</p>
<table summary= \"operation modes\" border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<thead>
  <tr>
    <th></th>
    <th>Plant</th>
    <th>Chiller</th>
    <th>Tank</th>
    <th>Flow direction</th>
    <th>Tank flow rate setpoint</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td>1.</td>
    <td>offline</td>
    <td>off</td>
    <td>off</td>
    <td>N/A</td>
    <td>0</td>
  </tr>
  <tr>
    <td>2.</td>
    <td>offline</td>
    <td>on</td>
    <td>charging</td>
    <td>N/A</td>
    <td>-1</td>
  </tr>
  <tr>
    <td>3.</td>
    <td>online</td>
    <td>on</td>
    <td>charging</td>
    <td>normal</td>
    <td>-1</td>
  </tr>
  <tr>
    <td>4.</td>
    <td>online</td>
    <td>on</td>
    <td>off</td>
    <td>normal</td>
    <td>0</td>
  </tr>
  <tr>
    <td>5.</td>
    <td>online</td>
    <td>on</td>
    <td>discharging</td>
    <td>normal</td>
    <td>1</td>
  </tr>
  <tr>
    <td>6.</td>
    <td>online</td>
    <td>off</td>
    <td>discharging</td>
    <td>normal</td>
    <td>1</td>
  </tr>
  <tr>
    <td>7.</td>
    <td>online</td>
    <td>off</td>
    <td>charging</td>
    <td>reverse</td>
    <td>-1</td>
  </tr>
</tbody>
</table>
</html>", revisions="<html>
<ul>
<li>
April 21, 2022 by Hongxiang Fu:<br/>
First implementation. This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2859\">#2859</a>.
</li>
</ul>
</html>"));
end RemoteChargingSchedule;
