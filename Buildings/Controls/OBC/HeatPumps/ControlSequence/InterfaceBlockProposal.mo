within ControlSequence;
model InterfaceBlockProposal
  Buildings.Controls.OBC.ASHRAE.G36.AHUs.MultiZone.VAV.Controller mulAHUCon
    annotation (Placement(transformation(extent={{-92,-58},{-52,30}})));
  Buildings.Controls.OBC.ASHRAE.G36.TerminalUnits.Reheat.Controller rehBoxCon
    annotation (Placement(transformation(extent={{72,48},{92,88}})));
  DXCoilStage.DXCoilCom DXCoilCom
    annotation (Placement(transformation(extent={{-22,-38},{-2,-18}})));
  AuxiliaryCoil.AuxCoilCon AuxCoilCon
    annotation (Placement(transformation(extent={{18,-24},{38,-4}})));
  DefrostCycle.DefCycCon DefCycCon
    annotation (Placement(transformation(extent={{20,-80},{40,-60}})));
  Buildings.Controls.OBC.ASHRAE.G36.ThermalZones.Setpoints TZonSet
    annotation (Placement(transformation(extent={{-74,50},{-54,90}})));
  CompressorDR.ComDRCon ComDRCon
    annotation (Placement(transformation(extent={{-22,58},{-2,78}})));
equation
  connect(mulAHUCon.yCooCoi, DXCoilCom.uCooCoi) annotation (Line(points={{-50,
          -29},{-32,-29},{-32,-22},{-24,-22}}, color={0,0,127}));
  connect(mulAHUCon.yHeaCoi, AuxCoilCon.HeaCoiCom) annotation (Line(points={{
          -50,-32},{-32,-32},{-32,-44},{10,-44},{10,-16},{16,-16}}, color={0,0,
          127}));
  connect(mulAHUCon.TAirSupSet, AuxCoilCon.TSupHeaSet) annotation (Line(points=
          {{-50,20},{10,20},{10,-8},{16,-8}}, color={0,0,127}));
  connect(DXCoilCom.yCoi, DefCycCon.uCoiAva) annotation (Line(points={{0,-34},{
          12,-34},{12,-70},{18,-70}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end InterfaceBlockProposal;
