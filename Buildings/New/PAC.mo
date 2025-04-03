within ;
model PAC "In-room portable air cleaner"

  replaceable package Medium =
    Modelica.Media.Interfaces.PartialMedium "Medium in the component";

  parameter Real eff[Medium.nC](min=0, max=1) = 0.9997
    "Virus removal efficiency";

  parameter Integer nPACs(min=0) = 1
    "Number of PACs";

  parameter Real flowPAC(min=0) = 0.094
    "PAC flow rate";

  parameter Real kpow(min=0) = 50
    "Rated power";

  Modelica.Blocks.Interfaces.RealInput C "Zone concentration"
    annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
  Modelica.Blocks.Interfaces.RealOutput yC_flow "Concentration outflow"
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
  Modelica.Blocks.Interfaces.BooleanInput u "on/off"
    annotation (Placement(transformation(extent={{-140,-40},{-100,0}})));
  Modelica.Blocks.Interfaces.RealOutput yP_PAC "Power output"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput yE_PAC "Energy output"
    annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
  Modelica.Blocks.Math.BooleanToReal booleanToReal
    annotation (Placement(transformation(extent={{-80,-30},{-60,-10}})));
  Modelica.Blocks.Continuous.Integrator E_PAC(use_reset=false)
    annotation (Placement(transformation(extent={{60,-50},{80,-30}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a diss "Heat dissipation"
    annotation (Placement(transformation(extent={{94,-90},{114,-70}})));

protected
  Modelica.Blocks.Math.Gain pow(final k=nPACs*kpow) "power of PAC"
    annotation (Placement(transformation(extent={{-20,-30},{0,-10}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prePow(final alpha=0)
    "Prescribed power (=heat and flow work) flow for dynamic model"
    annotation (Placement(transformation(extent={{10,-70},{30,-50}})));

equation
  connect(u, booleanToReal.u)
    annotation (Line(points={{-120,-20},{-82,-20}}, color={255,0,255}));
  connect(booleanToReal.y, pow.u)
    annotation (Line(points={{-59,-20},{-22,-20}}, color={0,0,127}));
  connect(pow.y,yP_PAC)  annotation (Line(points={{1,-20},{96,-20},{96,0},{110,
          0}}, color={0,0,127}));
  connect(pow.y, prePow.Q_flow) annotation (Line(points={{1,-20},{8,-20},{8,-46},
          {2,-46},{2,-60},{10,-60}}, color={0,0,127}));
  connect(prePow.port, diss) annotation (Line(points={{30,-60},{90,-60},{90,-80},
          {104,-80}}, color={191,0,0}));
  connect(pow.y,E_PAC. u) annotation (Line(points={{1,-20},{8,-20},{8,-40},{58,
          -40}}, color={0,0,127}));
  connect(E_PAC.y,yE_PAC)
    annotation (Line(points={{81,-40},{110,-40}}, color={0,0,127}));

  //for i in 1:Medium.nC loop
   yC_flow = booleanToReal.y*(-flowPAC)*eff[2]*nPACs*C;
  //end for;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-30,68},{32,50}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.None),
        Ellipse(
          extent={{-30,-42},{32,-60}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.None),
        Line(points={{-30,60},{-30,-50}}, color={28,108,200}),
        Line(points={{32,60},{32,-50}}, color={28,108,200})}), uses(Modelica(
          version="4.0.0")));
end PAC;
