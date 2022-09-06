within Buildings.Templates.Components.Valves.Interfaces;
partial model PartialValve "Interface class for valve"
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(
    final m_flow_nominal=dat.m_flow_nominal)
    annotation(__Linkage(enable=false));

  parameter Buildings.Templates.Components.Types.Valve typ
    "Equipment type"
    annotation (Evaluate=true, Dialog(group="Configuration"));
  final parameter Boolean is_none = typ == Buildings.Templates.Components.Types.Valve.None
    "Set to true if valve is none"
    annotation (Evaluate=true, Dialog(group="Configuration"));

  parameter Integer text_rotation = 0
    "Text rotation angle in icon layer"
    annotation(Dialog(tab="Graphics", enable=false));
  parameter Boolean text_flip = false
    "True to flip text horizontally in icon layer"
    annotation(Dialog(tab="Graphics", enable=false));

  parameter Buildings.Templates.Components.Data.Valve dat(final typ=typ)
    "Design and operating parameters";

  final parameter Modelica.Units.SI.PressureDifference dpValve_nominal=
    dat.dpValve_nominal
    "Nominal pressure drop of fully open valve";
  final parameter Modelica.Units.SI.PressureDifference dpFixed_nominal=
    dat.dpFixed_nominal
    "Nominal pressure drop of pipes and other equipment in flow leg";
  final parameter Modelica.Units.SI.PressureDifference dpFixedByp_nominal=
    dat.dpFixedByp_nominal
    "Nominal pressure drop in the bypass line";

  Modelica.Fluid.Interfaces.FluidPort_a portByp_a(
    redeclare final package Medium = Medium,
    p(start=Medium.p_default),
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0),
    h_outflow(start=Medium.h_default, nominal=Medium.h_default))
    if typ==Buildings.Templates.Components.Types.Valve.ThreeWayTwoPosition or
      typ==Buildings.Templates.Components.Types.Valve.ThreeWayModulating
    "Fluid connector with bypass line"
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
  Buildings.Templates.Components.Interfaces.Bus bus
    if not is_none
    "Control bus"
    annotation (Placement(
      transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={0,100}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={0,100})));
  annotation (
  Icon(coordinateSystem(preserveAspectRatio=false),
  graphics={
    Bitmap(
      visible=typ==Buildings.Templates.Components.Types.Valve.TwoWayModulating or
        typ==Buildings.Templates.Components.Types.Valve.ThreeWayModulating,
      extent=if text_flip then {{40,60},{-40,140}} else {{-40,60},{40,140}},
      rotation=text_rotation,
      fileName="modelica://Buildings/Resources/Images/Templates/Components/Actuators/Modulating.svg"),
    Bitmap(
      visible=typ==Buildings.Templates.Components.Types.Valve.TwoWayTwoPosition or
        typ==Buildings.Templates.Components.Types.Valve.ThreeWayTwoPosition,
      extent=if text_flip then {{40,60},{-40,140}} else {{-40,60},{40,140}},
      rotation=text_rotation,
      fileName="modelica://Buildings/Resources/Images/Templates/Components/Actuators/TwoPosition.svg"),
    Bitmap(
      visible=typ==Buildings.Templates.Components.Types.Valve.TwoWayTwoPosition or
        typ==Buildings.Templates.Components.Types.Valve.TwoWayModulating,
      extent={{-100,-100},{100,100}},
      fileName="modelica://Buildings/Resources/Images/Templates/Components/Valves/TwoWay.svg",
      rotation=-90),
    Bitmap(
      visible=typ==Buildings.Templates.Components.Types.Valve.ThreeWayTwoPosition or
        typ==Buildings.Templates.Components.Types.Valve.ThreeWayModulating,
      extent={{-100,-100},{100,100}},
      fileName="modelica://Buildings/Resources/Images/Templates/Components/Valves/ThreeWay.svg",
          rotation=-90),
    Bitmap(
      visible=typ==Buildings.Templates.Components.Types.Valve.None,
      extent={{-100,100},{100,-100}},
      fileName="modelica://Buildings/Resources/Images/Templates/Components/Valves/None.svg",
          rotation=-90)}),
    Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
This partial class provides a standard interface for valve models.
</p>
</html>"));
end PartialValve;
