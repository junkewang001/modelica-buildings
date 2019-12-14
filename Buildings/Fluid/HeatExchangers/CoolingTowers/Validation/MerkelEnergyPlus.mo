within Buildings.Fluid.HeatExchangers.CoolingTowers.Validation;
model MerkelEnergyPlus
  "Validation with EnergyPlus model for Merkel's cooling tower"
  extends Modelica.Icons.Example;

  package MediumAir = Buildings.Media.Air "Air medium model";
  package MediumWat = Buildings.Media.Water "Water medium model";

  parameter Modelica.SIunits.Density denAir=
    MediumAir.density(
      MediumAir.setState_pTX(MediumAir.p_default, MediumAir.T_default, MediumAir.X_default))
      "Default density of air";
  parameter Modelica.SIunits.Density denWat=
    MediumWat.density(
      MediumWat.setState_pTX(MediumWat.p_default, MediumWat.T_default, MediumWat.X_default))
      "Default density of water";

  // Cooling tower parameters
  parameter Modelica.SIunits.PressureDifference dp_nominal = 6000
    "Nominal pressure difference of cooling tower";
  parameter Modelica.SIunits.VolumeFlowRate vAir_flow_nominal = 5.382E-8
    "Nominal volumetric flow rate of air (medium 1)";  // 0.56054, from E+ output files; 5.382E-8, from E+ .idf
  parameter Modelica.SIunits.VolumeFlowRate vWat_flow_nominal = 2.76316E-5
    "Nominal volumetric flow rate of water (medium 2)";  // 0.00109181, from E+ output files; 2.76316E-5, from E+ .idf
  parameter Modelica.SIunits.MassFlowRate mAir_flow_nominal = vAir_flow_nominal * denAir
    "Nominal mass flow rate of air (medium 1)";
  parameter Modelica.SIunits.MassFlowRate mWat_flow_nominal = vWat_flow_nominal * denWat
    "Nominal mass flow rate of water (medium 2)";
  parameter Modelica.SIunits.Temperature TAirInWB_nominal = 20.59+273.15
    "Nominal outdoor wetbulb temperature";
  parameter Modelica.SIunits.Temperature TWatIn_nominal = 34.16+273.15
    "Nominal water inlet temperature";
  parameter Modelica.SIunits.Temperature TWatOut_initial = 33.019+273.15
    "Nominal water inlet temperature";
  parameter Modelica.SIunits.TemperatureDifference TApp_design = 8.9
    "Design approach temperature, from E+ .idf";
  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal = 20286.37455
    "Nominal heat transfer, positive";                              //20286.37455    25360.6
  parameter Modelica.SIunits.Power PFan_nominal = 213.00693
    "Nominal fan power";

  Merkel tow(
    redeclare package Medium = MediumWat,
    dp_nominal=dp_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_start=TWatOut_initial,
    m1_flow_nominal=mAir_flow_nominal,
    m2_flow_nominal=mWat_flow_nominal,
    TAirInWB_nominal=TAirInWB_nominal,
    TWatIn_nominal=TWatIn_nominal,
    Q_flow_nominal=Q_flow_nominal,
    PFan_nominal=PFan_nominal,
    configuration=Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow,
    yMin=0.01,
    fraFreCon=0.1,
    UACor(FRAirMin=0.2))
    "Merkel-theory based cooling tower"
    annotation (Placement(transformation(extent={{40,-40},{60,-20}})));

  Sources.MassFlowSource_T souWat(
    redeclare package Medium = MediumWat,
    use_m_flow_in=true,
    T=328.15,
    nPorts=1,
    use_T_in=true)
    "Water source to the cooling tower"
    annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));

  Sources.Boundary_pT sinWat(redeclare package Medium = MediumWat,nPorts=1)
    "Water sink from the cooling tower"
    annotation (Placement(transformation(extent={{100,-40},{80,-20}})));

  Modelica.Blocks.Sources.CombiTimeTable datRea(
    tableOnFile=true,
    fileName=ModelicaServices.ExternalReferences.loadResource(
      "modelica://Buildings//Resources/Data/Fluid/HeatExchangers/CoolingTowers/Validation/MerkelEnergyPlus/modelica.csv"),
    columns=2:12,
    tableName="modelica",
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments)
    "Reader for \"CoolingTower_VariableSpeed_Merkel.idf\" energy plus example results"
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));

  Controls.OBC.UnitConversions.From_degC TEntWat
    "Block that converts entering water temperature"
    annotation (Placement(transformation(extent={{-60,-44},{-40,-24}})));

  Controls.OBC.UnitConversions.From_degC TAirWB
    "Block that converts entering air wetbulb temperature"
    annotation (Placement(transformation(extent={{-60,20},{-40,40}})));

  Modelica.Blocks.Math.Division conFan
    "Block to convert fan power reading to fan control signal y"
    annotation (Placement(transformation(extent={{-20,54},{0,74}})));

  Modelica.Blocks.Sources.RealExpression PFan_nom(y=PFan_nominal)
    "Nominal fan power"
    annotation (Placement(transformation(extent={{-60,48},{-40,68}})));

  Modelica.Blocks.Sources.RealExpression TLvg_EP(y=datRea.y[6])
    "EnergyPlus results: cooling tower leaving water temperature"
    annotation (Placement(transformation(extent={{80,80},{100,100}})));

  Modelica.Blocks.Sources.RealExpression Q_flow_EP(y=-1*datRea.y[8])
    "EnergyPlus results: cooling tower heat flow rate"
    annotation (Placement(transformation(extent={{80,60},{100,80}})));

  Modelica.Blocks.Sources.RealExpression PFan_EP(y=datRea.y[9])
    "EnergyPlus results: fan power consumption"
    annotation (Placement(transformation(extent={{80,40},{100,60}})));

  Modelica.Blocks.Sources.RealExpression TOutAirDB(y=datRea.y[1])
    "Outdoor air drybulb temp"
    annotation (Placement(transformation(extent={{40,80},{60,100}})));

equation
  connect(tow.TAir, TAirWB.y)
    annotation (Line(points={{38,-26},{20,-26},{20,30},{-38,30}},
      color={0,0,127}));
  connect(souWat.ports[1], tow.port_a)
    annotation (Line(points={{0,-30},{40,-30}}, color={0,127,255}));
  connect(tow.port_b, sinWat.ports[1])
    annotation (Line(points={{60,-30},{80,-30}}, color={0,127,255}));
  connect(TEntWat.y, souWat.T_in)
    annotation (Line(points={{-38,-34},{-24,-34},{-24,-26},{-22,-26}},
      color={0,0,127}));
  connect(PFan_nom.y, conFan.u2)
    annotation (Line(points={{-39,58},{-22,58}}, color={0,0,127}));
  connect(conFan.y, tow.y)
    annotation (Line(points={{1,64},{30,64},{30,-22},{38,-22}},
      color={0,0,127}));
  connect(datRea.y[9], conFan.u1)
    annotation (Line(points={{-79,70},{-22,70}}, color={0,0,127}));
  connect(datRea.y[2], TAirWB.u)
    annotation (Line(points={{-79,70},{-70,70},{-70,30},{-62,30}},
      color={0,0,127}));
  connect(datRea.y[5], TEntWat.u)
    annotation (Line(points={{-79,70},{-70,70},{-70,-34},{-62,-34}},
      color={0,0,127}));
  connect(datRea.y[7], souWat.m_flow_in)
    annotation (Line(points={{-79,70},{-70,70},{-70,0},{-30,0},{-30,-22},{-22,-22}},
      color={0,0,127}));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false,
        extent={{-120,-120},{120,120}})),
    __Dymola_Commands(file=
        "modelica://Buildings/Resources/Scripts/Dymola/Fluid/HeatExchangers/CoolingTowers/Validation/MerkelEnergyPlus.mos"
        "Simulate and plot"),
    experiment(
      StartTime=0,
      StopTime=86400,
      Tolerance=1e-06),
    Documentation(info="<html>
<p>
This model validates the model
<a href=\"modelica://Buildings.Fluid.HeatExchangers.CoolingTowers.Merkel\">
Buildings.Fluid.HeatExchangers.CoolingTowers.Merkel</a> by comparing against results 
obtained from EnergyPlus 9.2.
</p>
<p>
The EnergyPlus results were obtained using the example file <code>CoolingTower:VariableSpeed</code>, 
with the cooling tower evaluated as the <code>CoolingTower:VariableSpeed:Merkel</code> model from 
EnergyPlus 9.2. 
</p>
</html>", revisions="<html>
<ul>
<li>
October 23, 2019 by Kathryn Hinkelman:<br/>
First implementation.
</li>
</ul>
</html>"));
end MerkelEnergyPlus;
