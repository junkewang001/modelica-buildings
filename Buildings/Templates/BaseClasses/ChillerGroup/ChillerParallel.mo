within Buildings.Templates.BaseClasses.ChillerGroup;
model ChillerParallel
  extends Buildings.Templates.Interfaces.ChillerGroup(
    final typ=Buildings.Templates.Types.ChillerGroup.ChillerParallel);

  inner replaceable Buildings.Templates.BaseClasses.Chiller.ElectricChiller chi[num]
    constrainedby Buildings.Templates.Interfaces.Chiller(
      redeclare each final package Medium1 = Medium1,
      redeclare each final package Medium2 = Medium2,
      final is_airCoo=is_airCoo)
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=0)));

  Fluid.Sources.MassFlowSource_T floZerChi_b(
    redeclare final package Medium = Medium,
    final m_flow=0,
    nPorts=1) "Zero flow boundary condition"
    annotation (Placement(transformation(extent={{-60,-86},{-40,-66}})));
  Experimental.DHC.EnergyTransferStations.BaseClasses.CollectorDistributor colDisChi(
    redeclare final package Medium = Medium,
    final mCon_flow_nominal=fill(m_flow_nominal, num),
    final nCon=num) "Chilled water side collector distributor"
    annotation (Placement(transformation(extent={{-20,-80},{20,-60}})));
  Fluid.Sources.MassFlowSource_T floZerChi_a(
    redeclare final package Medium = Medium,
    final m_flow=0,
    nPorts=1) "Zero flow boundary condition"
    annotation (Placement(transformation(extent={{60,-60},{40,-80}})));
  Fluid.Sources.MassFlowSource_T floZerCon_a(
    redeclare final package Medium = Medium,
    final m_flow=0,
    nPorts=1) "Zero flow boundary condition"
    annotation (Placement(transformation(extent={{-60,86},{-40,66}})));
  Experimental.DHC.EnergyTransferStations.BaseClasses.CollectorDistributor colDisCon(
    redeclare final package Medium = Medium,
    final mCon_flow_nominal=fill(m_flow_nominal, num),
    final nCon=num) "Condenser water side collector distributor"
    annotation (Placement(transformation(extent={{-20,80},{20,60}})));
  Fluid.Sources.MassFlowSource_T floZerCon_b(
    redeclare final package Medium = Medium,
    final m_flow=0,
    nPorts=1) "Zero flow boundary condition"
    annotation (Placement(transformation(extent={{60,60},{40,80}})));

equation
  connect(floZerCon_a.ports[1], colDisCon.port_bDisRet)
    annotation (Line(points={{-40,76},{-20,76}}, color={0,127,255}));
  connect(colDisCon.port_bDisSup, floZerCon_b.ports[1])
    annotation (Line(points={{20,70},{40,70}}, color={0,127,255}));
  connect(port_a1, colDisCon.port_aDisSup) annotation (Line(points={{-100,60},{-30,
          60},{-30,70},{-20,70}}, color={0,127,255}));
  connect(colDisCon.port_aDisRet, port_b1) annotation (Line(points={{20,76},{30,
          76},{30,60},{100,60}}, color={0,127,255}));
  connect(floZerChi_b.ports[1], colDisChi.port_bDisRet)
    annotation (Line(points={{-40,-76},{-20,-76}}, color={0,127,255}));
  connect(colDisChi.port_bDisSup, floZerChi_a.ports[1])
    annotation (Line(points={{20,-70},{40,-70}}, color={0,127,255}));
  connect(colDisChi.port_aDisSup, port_b2) annotation (Line(points={{-20,-70},{
          -32,-70},{-32,-60},{-100,-60}}, color={0,127,255}));
  connect(colDisChi.port_aDisRet, port_a2) annotation (Line(points={{20,-76},{
          30,-76},{30,-60},{100,-60}}, color={0,127,255}));

  connect(busCon.chi, chi.busCon) annotation (Line(
      points={{0.1,100.1},{0.1,90},{80,90},{80,30},{0,30},{0,20}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(colDisCon.ports_bCon, chi.port_a1) annotation (Line(points={{-12,60},{
          -12,40},{-40,40},{-40,12},{-20,12}}, color={0,127,255}));
  connect(chi.port_b1, colDisCon.ports_aCon) annotation (Line(points={{20,12},{40,
          12},{40,40},{12,40},{12,60}}, color={0,127,255}));
  connect(chi.port_b2, colDisChi.ports_bCon) annotation (Line(points={{-20,-12},
          {-40,-12},{-40,-40},{-12,-40},{-12,-60}}, color={0,127,255}));
  connect(chi.port_a2, colDisChi.ports_aCon) annotation (Line(points={{20,-12},
          {40,-12},{40,-40},{12,-40},{12,-60}}, color={0,127,255}));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-70,80},{70,-80}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-99,64},{102,54}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-99,-56},{102,-66}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-99,-54},{102,-66}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-66},{0,-54}},
          lineColor={0,0,127},
          pattern=LinePattern.None,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-104,66},{98,54}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-2,54},{98,66}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-44,52},{-40,12}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-56,70},{58,52}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-42,2},{-52,12},{-32,12},{-42,2}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-42,2},{-52,-10},{-32,-10},{-42,2}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-44,-10},{-40,-50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{38,52},{42,-50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-56,-50},{58,-68}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{18,24},{62,-18}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{40,24},{22,-8},{58,-8},{40,24}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-99,-54},{102,-66}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-66},{0,-54}},
          lineColor={0,0,127},
          pattern=LinePattern.None,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-104,66},{98,54}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-2,54},{98,66}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-44,52},{-40,12}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-56,70},{58,52}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-42,2},{-52,12},{-32,12},{-42,2}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-42,2},{-52,-10},{-32,-10},{-42,2}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-44,-10},{-40,-50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{38,52},{42,-50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-56,-50},{58,-68}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{18,24},{62,-18}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{40,24},{22,-8},{58,-8},{40,24}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end ChillerParallel;
