within Buildings.Fluid.Chillers.BaseClasses;
partial model Carnot_Z
  extends Buildings.Fluid.Interfaces.PartialSixPortInterface_Z(
    m1_flow_nominal = QCon_flow_nominal/cp1_default/dTCon_nominal,
    m2_flow_nominal_GCon = QGro_flow_nominal_Con/cp2_default/dTGro_nominal_Con,
    m2_flow_nominal_GEva= QGro_flow_nominal_Eva/cp2_default/dTGro_nominal_Eva,
    m3_flow_nominal = QEva_flow_nominal/cp3_default/dTEva_nominal);
                                                                   // check the error for this line

  constant Boolean homotopyInitialization = true "= true, use homotopy method"
    annotation(HideResult=true);
  parameter Modelica.Units.SI.HeatFlowRate QCon_flow_nominal(min=0)
    "Nominal building cooling heat flow rate (QCon_flow_nominal > 0)"
    annotation (Dialog(group="Nominal condition"));

  parameter Modelica.Units.SI.HeatFlowRate QEva_flow_nominal(max=0)
    "Nominal building cooling heat flow rate (QEva_flow_nominal < 0)"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.Units.SI.HeatFlowRate QGro_flow_nominal_Con(min=0)
    "Nominal ground heating flow rate" annotation (Dialog(group="Nominal condition"));

  parameter Modelica.Units.SI.HeatFlowRate QGro_flow_nominal_Eva(max=0)
    "Nominal ground cooling flow rate" annotation (Dialog(group="Nominal condition"));

  parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal(final max=0)=
       -10 "Temperature difference evaporator outlet-inlet"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal(final min=0)=
       10 "Temperature difference condenser outlet-inlet"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.Units.SI.TemperatureDifference dTGro_nominal_Eva(final max=0)=
       -10 "Temperature difference ground outlet-inlet when work as evaporator"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.Units.SI.TemperatureDifference dTGro_nominal_Con(final min=0)=
       10 "Temperature difference ground outlet-inlet when work as condenser"
    annotation (Dialog(group="Nominal condition"));

  // Efficiency
  parameter Boolean use_eta_Carnot_nominal = true
    "Set to true to use Carnot effectiveness etaCarnot_nominal rather than COP_nominal"
    annotation(Dialog(group="Efficiency"));

  parameter Real etaCarnot_nominal_ConGro(unit="1") = COP_nominal_ConGro/
    (TUseAct_nominal_ConGro/(TCon_nominal+TAppCon_nominal - (TGro_nominal-TAppGro_nominal)))
    "Carnot effectiveness for condense-ground(=COP/COP_Carnot) used if use_eta_Carnot_nominal = true"
    annotation (Dialog(group="Efficiency", enable=use_eta_Carnot_nominal));


  parameter Real COP_nominal_ConGro(unit="1") = etaCarnot_nominal_ConGro*TUseAct_nominal_ConGro/
    (TCon_nominal+TAppCon_nominal - (TGro_nominal-TAppGro_nominal))
    "Coefficient of performance at TEva_nominal and TCon_nominal, used if use_eta_Carnot_nominal = false"
    annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));

  parameter Real etaCarnot_nominal_GroEva(unit="1") = COP_nominal_GroEva/
    (TUseAct_nominal_GroEva/(TGro_nominal+TAppGro_nominal - (TEva_nominal-TAppEva_nominal)))
    "Carnot effectiveness for condense-ground(=COP/COP_Carnot) used if use_eta_Carnot_nominal = true"
    annotation (Dialog(group="Efficiency", enable=use_eta_Carnot_nominal));
  parameter Real COP_nominal_GroEva(unit="1") = etaCarnot_nominal_GroEva*TUseAct_nominal_GroEva/
    (TGro_nominal+TAppGro_nominal - (TEva_nominal-TAppEva_nominal))
    "Coefficient of performance at TEva_nominal and TGro_nominal, used if use_eta_Carnot_nominal = false"
    annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));

  parameter Modelica.Units.SI.Temperature TCon_nominal=303.15
    "Condenser temperature used to compute COP_nominal if use_eta_Carnot_nominal=false"
    annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));
  parameter Modelica.Units.SI.Temperature TEva_nominal=278.15
    "Evaporator temperature used to compute COP_nominal if use_eta_Carnot_nominal=false"
    annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));
  parameter Modelica.Units.SI.Temperature TGro_nominal=293.15
    "Evaporator temperature used to compute COP_nominal if use_eta_Carnot_nominal=false"
    annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));

  parameter Real a[:] = {1}
    "Coefficients for efficiency curve (need p(a=a, yPL=1)=1)"
    annotation (Dialog(group="Efficiency"));

  parameter Modelica.Units.SI.Pressure dp1_nominal(displayUnit="Pa")
    "Pressure difference over condenser"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.Units.SI.Pressure dp2_nominal_gcon(displayUnit="Pa")
    "Pressure difference over ground condenser"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.Units.SI.Pressure dp2_nominal_geva(displayUnit="Pa")
    "Pressure difference over ground evaporator"
    annotation (Dialog(group="Nominal condition"));

  parameter Modelica.Units.SI.Pressure dp3_nominal(displayUnit="Pa")
    "Pressure difference over groud"
    annotation (Dialog(group="Nominal condition"));

  parameter Modelica.Units.SI.TemperatureDifference TAppCon_nominal(min=0)=
    if cp1_default < 1500 then 5 else 2
    "Temperature difference between refrigerant and working fluid outlet in condenser"
    annotation (Dialog(group="Efficiency"));

  parameter Modelica.Units.SI.TemperatureDifference TAppEva_nominal(min=0)=
    if cp2_default < 1500 then 5 else 2
    "Temperature difference between refrigerant and working fluid outlet in evaporator"
    annotation (Dialog(group="Efficiency"));
  parameter Modelica.Units.SI.TemperatureDifference TAppGro_nominal(min=0)=
    if cp3_default < 1500 then 5 else 2
    "Temperature difference between refrigerant and working fluid outlet in ground"
    annotation (Dialog(group="Efficiency"));

  parameter Boolean from_dp1=false
    "= true, use m_flow = f(dp) else dp = f(m_flow)"
    annotation (Dialog(tab="Flow resistance", group="Condenser"));


  parameter Boolean from_dp2_geva=false
    "= true, use m_flow = f(dp) else dp = f(m_flow)"
    annotation (Dialog(tab="Flow resistance", group="Ground Evaporator"));

  parameter Boolean from_dp2_gcon=false
    "= true, use m_flow = f(dp) else dp = f(m_flow)"
    annotation (Dialog(tab="Flow resistance", group="Ground Condenser"));



  parameter Boolean from_dp3=false
    "= true, use m_flow = f(dp) else dp = f(m_flow)"
    annotation (Dialog(tab="Flow resistance", group="Evaporator"));

  parameter Boolean linearizeFlowResistance1=false
    "= true, use linear relation between m_flow and dp for any flow rate"
    annotation (Dialog(tab="Flow resistance", group="Condenser"));
  parameter Boolean linearizeFlowResistance2_geva=false
    "= true, use linear relation between m_flow and dp for any flow rate"
    annotation (Dialog(tab="Flow resistance", group="Ground Evaporator"));
  parameter Boolean linearizeFlowResistance2_gcon=false
    "= true, use linear relation between m_flow and dp for any flow rate"
    annotation (Dialog(tab="Flow resistance", group="Ground Condenser"));

  parameter Boolean linearizeFlowResistance3=false
    "= true, use linear relation between m_flow and dp for any flow rate"
    annotation (Dialog(tab="Flow resistance", group="Evaporator"));
  parameter Real deltaM1(final unit="1")=0.1
    "Fraction of nominal flow rate where flow transitions to laminar"
    annotation (Dialog(tab="Flow resistance", group="Condenser"));
  parameter Real deltaM2_geva(final unit="1")=0.1
    "Fraction of nominal flow rate where flow transitions to laminar"
    annotation (Dialog(tab="Flow resistance", group="Ground Evaporator"));

  parameter Real deltaM2_gcon(final unit="1")=0.1
    "Fraction of nominal flow rate where flow transitions to laminar"
    annotation (Dialog(tab="Flow resistance", group="Ground Condenser"));






  parameter Real deltaM3(final unit="1")=0.1
    "Fraction of nominal flow rate where flow transitions to laminar"
    annotation (Dialog(tab="Flow resistance", group="Evaporator"));

  parameter Modelica.Units.SI.Time tau1=60
    "Time constant at nominal flow rate (used if energyDynamics1 <> Modelica.Fluid.Types.Dynamics.SteadyState)"
    annotation (Dialog(tab="Dynamics", group="Condenser"));
  parameter Modelica.Units.SI.Time tau2_gcon=60
    "Time constant at nominal flow rate (used if energyDynamics2 <> Modelica.Fluid.Types.Dynamics.SteadyState)"
    annotation (Dialog(tab="Dynamics", group="Ground Condenser"));

  parameter Modelica.Units.SI.Time tau2_geva=60
    "Time constant at nominal flow rate (used if energyDynamics2 <> Modelica.Fluid.Types.Dynamics.SteadyState)"
    annotation (Dialog(tab="Dynamics", group="Ground Evaporator"));

  parameter Modelica.Units.SI.Time tau3=60
    "Time constant at nominal flow rate (used if energyDynamics2 <> Modelica.Fluid.Types.Dynamics.SteadyState)"
    annotation (Dialog(tab="Dynamics", group="Evaporator"));
  parameter Modelica.Units.SI.Temperature T1_start=Medium1.T_default
    "Initial or guess value of set point"
    annotation (Dialog(tab="Dynamics", group="Ground"));
  parameter Modelica.Units.SI.Temperature T2_start_geva=Medium2.T_default
    "Initial or guess value of set point"
    annotation (Dialog(tab="Dynamics", group="Ground Evaporator"));
  parameter Modelica.Units.SI.Temperature T2_start_gcon=Medium2.T_default
    "Initial or guess value of set point"
    annotation (Dialog(tab="Dynamics", group="Ground Condenser"));
  parameter Modelica.Units.SI.Temperature T3_start=Medium3.T_default
    "Initial or guess value of set point"
    annotation (Dialog(tab="Dynamics", group="Ground"));
  parameter Modelica.Fluid.Types.Dynamics energyDynamics=
    Modelica.Fluid.Types.Dynamics.SteadyState "Type of energy balance: dynamic (3 initialization options) or steady state"
    annotation (Dialog(tab="Dynamics", group="Evaporator and condenser"));

  Modelica.Blocks.Interfaces.RealOutput QCon_flow(
    final quantity="HeatFlowRate",
    final unit="W") "Actual heating heat flow rate added to fluid 1"
    annotation (Placement(transformation(extent={{108,32},{118,44}}),
        iconTransformation(extent={{108,32},{118,44}})));

  Modelica.Blocks.Interfaces.RealOutput P_ConGro(
    final quantity="Power",
    final unit="W") "Electric power consumed by compressor ConGro"
    annotation (Placement(transformation(extent={{106,90},{116,102}}),
        iconTransformation(extent={{106,90},{116,102}})));

  Modelica.Blocks.Interfaces.RealOutput QEva_flow(
    final quantity="HeatFlowRate",
    final unit="W") "Actual cooling heat flow rate removed from fluid 2"
    annotation (Placement(transformation(extent={{110,-46},{120,-34}}),
        iconTransformation(extent={{110,-46},{120,-34}})));
  Modelica.Blocks.Interfaces.RealOutput QGro_flow_Eva(final quantity="HeatFlowRate",
      final unit="W") "Actual cooling heat flow rate removed from fluid 2"
    annotation (Placement(transformation(extent={{120,-10},{140,10}}),
        iconTransformation(extent={{108,14},{118,26}})));
  Modelica.Blocks.Interfaces.RealOutput P_GroEva(final quantity="Power",
      final unit="W")
                    "Electric power consumed by compressor GroEva"
    annotation (Placement(transformation(extent={{120,80},{140,100}}),
        iconTransformation(extent={{106,74},{116,86}})));

  Real yPL_ConGro(final unit="1", min=0) = QCon_flow/QCon_flow_nominal "Part load ratio";

  Real yPL_GroEva(final unit="1", min=0) = QEva_flow/QEva_flow_nominal
      "Part load ratio";

  Real etaPL_ConGro(final unit = "1")=
    if evaluate_etaPL
      then Buildings.Utilities.Math.Functions.polynomial(a=a, x=yPL_ConGro)
      else 1
    "Efficiency due to part load (etaPL(yPL=1)=1)";
  Real etaPL_GroEva(final unit = "1")=
    if evaluate_etaPL
      then Buildings.Utilities.Math.Functions.polynomial(a=a, x=yPL_GroEva)
      else 1
    "Efficiency due to part load (etaPL(yPL=1)=1)";
  Real COP_ConGro(min=0, final unit="1") = etaCarnot_nominal_internal_ConGro * COPCar_ConGro * etaPL_ConGro
    "Coefficient of performance";
  Real COP_GroEva(min=0, final unit="1") = etaCarnot_nominal_internal_GroEva * COPCar_GroEva * etaPL_GroEva
    "Coefficient of performance";

  Real COPCar_ConGro(min=0) = TUseAct_ConGro/Buildings.Utilities.Math.Functions.smoothMax(
    x1=1,
    x2=TConAct - TGroAct_Eva,
    deltaX=0.25) "Carnot efficiency";

  Real COPCar_GroEva(min=0) = TUseAct_GroEva/Buildings.Utilities.Math.Functions.smoothMax(
    x1=1,
    x2=TGroAct_Con - TEvaAct,
    deltaX=0.25) "Carnot efficiency";

  Modelica.Units.SI.Temperature TConAct(start=TCon_nominal + TAppCon_nominal)=
       Medium1.temperature(staB1) + QCon_flow/QCon_flow_nominal*TAppCon_nominal
    "Condenser temperature used to compute efficiency, taking into account pinch temperature between fluid and refrigerant";

  Modelica.Units.SI.Temperature TEvaAct(start=TEva_nominal - TAppEva_nominal)=
       Medium3.temperature(staB3) - QEva_flow/QEva_flow_nominal*TAppEva_nominal
    "Evaporator temperature used to compute efficiency, taking into account pinch temperature between fluid and refrigerant";


  Modelica.Units.SI.Temperature TGroAct_Eva(start=TGro_nominal - TAppGro_nominal)=
       Medium2.temperature(staA2) - QGro_flow_Eva/QEva_flow_nominal*TAppEva_nominal
    "Ground Evaporator temperature used to compute efficiency, taking into account pinch temperature between fluid and refrigerant";


  Modelica.Units.SI.Temperature TGroAct_Con(start=TGro_nominal + TAppGro_nominal)=
       Medium2.temperature(staB2) + QGro_flow_Con/QCon_flow_nominal*TAppCon_nominal
    "Ground Condenser temperature used to compute efficiency, taking into account pinch temperature between fluid and refrigerant";


  Modelica.Blocks.Interfaces.RealOutput QGro_flow_Con(final quantity=
        "HeatFlowRate", final unit="W")
    "Actual heating heat flow rate removed from fluid 2" annotation (Placement(
        transformation(extent={{120,-38},{140,-18}}), iconTransformation(extent=
           {{110,-28},{120,-16}})));
protected
  parameter Real etaCarnot_nominal_internal_ConGro(unit="1")=
    if use_eta_Carnot_nominal
      then etaCarnot_nominal_ConGro
      else COP_nominal_ConGro/
           (TUseAct_nominal_ConGro / (TCon_nominal + TAppCon_nominal - (TGro_nominal - TAppGro_nominal)))
    "Carnot effectiveness (=COP/COP_Carnot) used to compute COP";

  parameter Real etaCarnot_nominal_internal_GroEva(unit="1")=
    if use_eta_Carnot_nominal
      then etaCarnot_nominal_GroEva
      else COP_nominal_GroEva/
           (TUseAct_nominal_GroEva / (TGro_nominal + TAppGro_nominal - (TEva_nominal - TAppEva_nominal)))
    "Carnot effectiveness (=COP/COP_Carnot) used to compute COP";


  // For Carnot_y, computing etaPL = f(yPL) introduces a nonlinear equation.
  // The parameter below avoids this if a = {1}.
  final parameter Boolean evaluate_etaPL=
    not ((size(a, 1) == 1 and abs(a[1] - 1)  < Modelica.Constants.eps))
    "Flag, true if etaPL should be computed as it depends on yPL"
    annotation(Evaluate=true);


  final parameter Modelica.Units.SI.Temperature TUseAct_nominal_GroEva=TEva_nominal - TAppEva_nominal
    "Nominal evaporator temperature for chiller or condenser temperature for heat pump, taking into account pinch temperature between fluid and refrigerant";

  final parameter Modelica.Units.SI.Temperature TUseAct_nominal_ConGro= TCon_nominal
       + TAppCon_nominal
    "Nominal evaporator temperature for chiller or condenser temperature for heat pump, taking into account pinch temperature between fluid and refrigerant";



  Modelica.Units.SI.Temperature TUseAct_GroEva= TEvaAct
    "Temperature of useful heat (evaporator for chiller, condenser for heat pump), taking into account pinch temperature between fluid and refrigerant";

  Modelica.Units.SI.Temperature TUseAct_ConGro=TConAct
    "Temperature of useful heat (evaporator for chiller, condenser for heat pump), taking into account pinch temperature between fluid and refrigerant";

  final parameter Modelica.Units.SI.SpecificHeatCapacity cp1_default=
      Medium1.specificHeatCapacityCp(Medium1.setState_pTX(
      p=Medium1.p_default,
      T=Medium1.T_default,
      X=Medium1.X_default))
    "Specific heat capacity of medium 1 at default medium state";

  final parameter Modelica.Units.SI.SpecificHeatCapacity cp2_default=
      Medium2.specificHeatCapacityCp(Medium2.setState_pTX(
      p=Medium2.p_default,
      T=Medium2.T_default,
      X=Medium2.X_default))
    "Specific heat capacity of medium 2 at default medium state";
  final parameter Modelica.Units.SI.SpecificHeatCapacity cp3_default=
      Medium3.specificHeatCapacityCp(Medium3.setState_pTX(
      p=Medium3.p_default,
      T=Medium3.T_default,
      X=Medium3.X_default))
    "Specific heat capacity of medium 3 at default medium state";
  Medium1.ThermodynamicState staA1 = Medium1.setState_phX(
    port_a1.p,
    inStream(port_a1.h_outflow),
    inStream(port_a1.Xi_outflow)) "Medium properties in port_a1";

  Medium1.ThermodynamicState staB1 = Medium1.setState_phX(
    port_b1.p,
    port_b1.h_outflow,
    port_b1.Xi_outflow) "Medium properties in port_b1";

  Medium2.ThermodynamicState staA2 = Medium2.setState_phX(
    port_a2.p,
    inStream(port_a2.h_outflow),
    inStream(port_a2.Xi_outflow)) "Medium properties in port_a2";


   Medium2.ThermodynamicState staB2_Eva = Medium2.setState_phX(
    port_b2.p,
    port_b2.h_outflow,
    port_b2.Xi_outflow) "Medium properties in port_b2";


  Medium2.ThermodynamicState staB2_Con=Medium2.setState_phX(
      groascon.port_a.p,
      groascon.port_a.h_outflow,
      groascon.port_a.Xi_outflow) "Medium properties in port_b2";



  Medium3.ThermodynamicState staA3 = Medium3.setState_phX(
    port_a3.p,
    inStream(port_a3.h_outflow),
    inStream(port_a3.Xi_outflow)) "Medium properties in port_a3";

  Medium3.ThermodynamicState staB3 = Medium3.setState_phX(
    port_b3.p,
    port_b3.h_outflow,
    port_b3.Xi_outflow) "Medium properties in port_b3";
  replaceable Interfaces.PartialTwoPortInterface con
    constrainedby Interfaces.PartialTwoPortInterface(
      redeclare final package Medium = Medium1,
      final allowFlowReversal=allowFlowReversal1,
      final m_flow_nominal=m1_flow_nominal,
      final m_flow_small=m1_flow_small,
      final show_T=false) "Condenser"
    annotation (Placement(transformation(extent={{30,50},{50,70}})));

  replaceable Interfaces.PartialTwoPortInterface groascon constrainedby
    Interfaces.PartialTwoPortInterface(
    redeclare final package Medium = Medium2,
    final allowFlowReversal=allowFlowReversal2,
    final m_flow_nominal=m2_flow_nominal_GCon,
    final m_flow_small=m2_flow_small,
    final show_T=false) "Condenser for groundside"
    annotation (Placement(transformation(extent={{-14,-8},{-34,12}})));

  replaceable Interfaces.PartialTwoPortInterface eva constrainedby
    Interfaces.PartialTwoPortInterface(
    redeclare final package Medium = Medium3,
    final allowFlowReversal=allowFlowReversal3,
    final m_flow_nominal=m3_flow_nominal,
    final m_flow_small=m3_flow_small,
    final show_T=false) "Evaporator"
    annotation (Placement(transformation(extent={{-16,-74},{-36,-54}})));
  replaceable Interfaces.PartialTwoPortInterface groaseva constrainedby
    Interfaces.PartialTwoPortInterface(
    redeclare final package Medium = Medium2,
    final allowFlowReversal=allowFlowReversal2,
    final m_flow_nominal=m2_flow_nominal_GEva,
    final m_flow_small=m2_flow_small,
    final show_T=false) "Evaporator for groundside"
    annotation (Placement(transformation(extent={{48,-8},{28,12}})));
initial equation
  assert(dTEva_nominal < 0,
    "Parameter dTEva_nominal must be negative.");
  assert(dTCon_nominal > 0,
    "Parameter dTCon_nominal must be positive.");

  assert(abs(Buildings.Utilities.Math.Functions.polynomial(
         a=a, x=1)-1) < 0.01, "Efficiency curve is wrong. Need etaPL(y=1)=1.");
  assert(etaCarnot_nominal_internal_GroEva < 1,   "Parameters lead to etaCarnot_nominal > 1. Check parameters.");
  assert(etaCarnot_nominal_internal_ConGro < 1,   "Parameters lead to etaCarnot_nominal > 1. Check parameters.");

  assert(homotopyInitialization, "In " + getInstanceName() +
    ": The constant homotopyInitialization has been modified from its default value. This constant will be removed in future releases.",
    level = AssertionLevel.warning);

equation
  connect(port_a1, con.port_a)
    annotation (Line(points={{-100,60},{30,60}},           color={0,127,255}));
  connect(con.port_b, port_b1)
    annotation (Line(points={{50,60},{100,60}},         color={0,127,255}));

  connect(port_b3, eva.port_b) annotation (Line(points={{-96,-66},{-94,-66},{
          -94,-64},{-36,-64}},
                           color={0,127,255}));
  connect(eva.port_a, port_a3) annotation (Line(points={{-16,-64},{86,-64},{86,
          -62},{100,-62}},
                      color={0,127,255}));
  connect(port_a2, groaseva.port_a) annotation (Line(points={{100,0},{98,0},{98,
          2},{48,2}}, color={0,127,255}));
  connect(groaseva.port_b, groascon.port_a)
    annotation (Line(points={{28,2},{-14,2}}, color={0,127,255}));
  connect(groascon.port_b, port_b2) annotation (Line(points={{-34,2},{-84,2},{-84,
          4},{-98,4}}, color={0,127,255}));
  connect(QGro_flow_Eva, QGro_flow_Eva)
    annotation (Line(points={{130,0},{130,0}}, color={0,0,127}));
  annotation (
  Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},
            {100,100}}),       graphics={
        Rectangle(
          extent={{-70,80},{70,-80}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-56,68},{58,50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-56,-52},{58,-70}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-103,64},{98,54}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={162,29,33},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-2,54},{98,64}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-101,-56},{100,-66}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-66},{0,-56}},
          lineColor={0,0,127},
          pattern=LinePattern.None,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{16,50},{20,36}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-40,-4},{-36,-20}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{62,0},{100,0}},                 color={0,0,255}),
        Rectangle(
          extent={{-56,14},{58,-4}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{16,24},{20,14}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{18,29.389},{11.056,36.611},{24.944,36.611},{18,29.389}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{18,29.7777},{11.336,22.2223},{24.664,22.2223},{18,29.7777}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{44,24},{48,14}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{44,50},{48,36}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{38,38},{54,22}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{46,37.0323},{39.039,26.9677},{52.961,26.9677},{46,37.0323}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-92,-2},{0,8}},
          lineColor={0,0,127},
          pattern=LinePattern.None,
          fillColor={255,170,85},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-1,8},{94,-2}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-38,-26.611},{-44.944,-19.389},{-31.056,-19.389},{-38,
              -26.611}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-14,-18.9677},{-20.961,-29.0323},{-7.039,-29.0323},{-14,
              -18.9677}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-16,-4},{-12,-20}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-16,-32},{-12,-52}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-40,-32},{-36,-52}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-38,-26.2223},{-44.664,-33.7777},{-31.336,-33.7777},{-38,
              -26.2223}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-22,-18},{-6,-34}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-14,-18.9677},{-20.961,-29.0323},{-7.039,-29.0323},{-14,
              -18.9677}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}),
      Documentation(info="<html>
<p>
This is the base class for the Carnot chiller and the Carnot heat pump
whose coefficient of performance COP changes
with temperatures in the same way as the Carnot efficiency changes.
</p>
<p>
The model allows to either specify the Carnot effectivness
<i>&eta;<sub>Carnot,0</sub></i>, or
a <i>COP<sub>0</sub></i>
at the nominal conditions, together with
the evaporator temperature <i>T<sub>eva,0</sub></i> and
the condenser temperature <i>T<sub>con,0</sub></i>, in which
case the model computes the Carnot effectivness as
</p>
<p align=\"center\" style=\"font-style:italic;\">
&eta;<sub>Carnot,0</sub> =
  COP<sub>0</sub>
&frasl;  (T<sub>use,0</sub> &frasl; (T<sub>con,0</sub>-T<sub>eva,0</sub>)),
</p>
<p>
where
<i>T<sub>use</sub></i> is the temperature of the the useful heat,
e.g., the evaporator temperature for a chiller or the condenser temperature
for a heat pump.
</p>
<p>
The COP is computed as the product
</p>
<p align=\"center\" style=\"font-style:italic;\">
  COP = &eta;<sub>Carnot,0</sub> COP<sub>Carnot</sub> &eta;<sub>PL</sub>,
</p>
<p>
where <i>COP<sub>Carnot</sub></i> is the Carnot efficiency and
<i>&eta;<sub>PL</sub></i> is the part load efficiency, expressed using
a polynomial.
This polynomial has the form
</p>
<p align=\"center\" style=\"font-style:italic;\">
  &eta;<sub>PL</sub> = a<sub>1</sub> + a<sub>2</sub> y + a<sub>3</sub> y<sup>2</sup> + ...
</p>
<p>
where <i>y &isin; [0, 1]</i> is
either the part load for cooling in case of a chiller, or the part load of heating in
case of a heat pump, and the coefficients <i>a<sub>i</sub></i>
are declared by the parameter <code>a</code>.
</p>
<h4>Implementation</h4>
<p>
To make this base class applicable to chiller or heat pumps, it uses
the boolean constant <code>COP_is_for_cooling</code>.
Depending on its value, the equations for the coefficient of performance
and the part load ratio are set up.
</p>
</html>", revisions="<html>
<ul>
<li>
April 14, 2020, by Michael Wetter:<br/>
Changed <code>homotopyInitialization</code> to a constant.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1341\">IBPSA, #1341</a>.
</li>
<li>
September 12, 2019, by Michael Wetter:<br/>
Corrected value of <code>evaluate_etaPL</code> and how it is used.
This correction only affects protected variables and does not affect the results.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1200\">
#1200</a>.
</li>
<li>
June 16, 2017, by Michael Wetter:<br/>
Added temperature difference between fluids in condenser and evaporator
for computation of nominal COP and effectiveness.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/698\">
#698</a>.
</li>
<li>
March 28, 2017, by Felix Buenning:<br/>
Added temperature difference between fluids in condenser and evaporator.
The difference is based on discussions with Emerson Climate Technologies.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/698\">
#698</a>.
</li>
<li>
January 2, 2017, by Filip Jorissen:<br/>
Removed option for choosing what temperature
should be used to compute the Carnot efficiency.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/497\">
issue 497</a>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
First implementation of this base class.
</li>
</ul>
</html>"));
end Carnot_Z;
