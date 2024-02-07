within Buildings.Fluid.Chillers.BaseClasses;
partial model PartialCarnot_y_Z
  "Partial chiller model with performance curve adjusted based on Carnot efficiency"
  extends Carnot_Z(
    final QGro_flow_nominal_Eva = -P_nominal_ConGro * (COP_nominal_ConGro-1),
    final QGro_flow_nominal_Con = P_nominal_GroEva -QEva_flow_nominal,
    final QCon_flow_nominal = P_nominal_ConGro - QGro_flow_nominal_Eva,
    final QEva_flow_nominal = -P_nominal_GroEva * COP_nominal_GroEva,
    redeclare HeatExchangers.HeaterCooler_u con(
      final from_dp=from_dp1,
      final dp_nominal=dp1_nominal,
      final linearizeFlowResistance=linearizeFlowResistance1,
      final deltaM=deltaM1,
      final tau=tau1,
      final T_start=T1_start,
      final energyDynamics=energyDynamics,
      final homotopyInitialization=homotopyInitialization,
      final Q_flow_nominal=QCon_flow_nominal),
      redeclare HeatExchangers.HeaterCooler_u congro(
      final from_dp=from_dp2_geva,
      final dp_nominal=dp2_nominal_geva,
      final linearizeFlowResistance=linearizeFlowResistance2_geva,
      final deltaM=deltaM2_geva,
      final tau=tau2_geva,
      final T_start=T2_start_geva,
      final energyDynamics=energyDynamics,
      final homotopyInitialization=homotopyInitialization,
      final Q_flow_nominal= QGro_flow_nominal_Eva),
      redeclare HeatExchangers.HeaterCooler_u groeva(
      final from_dp=from_dp2_gcon,
      final dp_nominal=dp2_nominal_gcon,
      final linearizeFlowResistance=linearizeFlowResistance2_gcon,
      final deltaM=deltaM2_gcon,
      final tau=tau2_gcon,
      final T_start=T2_start_gcon,
      final energyDynamics=energyDynamics,
      final homotopyInitialization=homotopyInitialization,
      final Q_flow_nominal= QGro_flow_nominal_Con),
      redeclare HeatExchangers.HeaterCooler_u eva(
      final from_dp=from_dp3,
      final dp_nominal=dp3_nominal,
      final linearizeFlowResistance=linearizeFlowResistance3,
      final deltaM=deltaM3,
      final tau=tau3,
      final T_start=T3_start,
      final energyDynamics=energyDynamics,
      final homotopyInitialization=homotopyInitialization,
      final Q_flow_nominal=QEva_flow_nominal));                               // Ground work as evaportor
                                                                      // Ground work as condenser

  parameter Modelica.Units.SI.Power P_nominal_GroEva(min=0)
    "Nominal compressor power (at y=1)"
    annotation (Dialog(group="Nominal condition"));

  parameter Modelica.Units.SI.Power P_nominal_ConGro(min=0)
    "Nominal compressor power (at y=1)"
    annotation (Dialog(group="Nominal condition"));

  Modelica.Blocks.Interfaces.RealInput y_GroEva(min=0, max=1, unit="1")
    "Part load ratio of compressor GroEva "
    annotation (Placement(transformation(extent={{-112,-38},{-92,-14}}),
        iconTransformation(extent={{-112,-38},{-92,-14}})));


  Modelica.Blocks.Interfaces.RealInput y_ConGro(
    min=0,
    max=1,
    unit="1")
    "Part load ratio of compressor ConGro"
    annotation (Placement(transformation(extent={{-110,18},{-90,40}}),
        iconTransformation(extent={{-110,18},{-90,40}})));
//protected
  Modelica.Units.SI.HeatFlowRate QCon_flow_internal(start=QCon_flow_nominal)=
    P_ConGro - QGro_flow_internal_Eva "Condenser heat input";

  Modelica.Units.SI.HeatFlowRate QEva_flow_internal(start=QEva_flow_nominal)=
     -COP_GroEva*P_GroEva  "Evaporator heat input";

  Modelica.Units.SI.HeatFlowRate QGro_flow_internal_Eva(start=QGro_flow_nominal_Eva)=
    (1 - COP_ConGro)*P_ConGro "Ground Evaporator part heat input";

  Modelica.Units.SI.HeatFlowRate QGro_flow_internal_Con(start=QGro_flow_nominal_Con)=
    P_GroEva-QEva_flow_internal "Ground Condenser part heat input";


  Modelica.Blocks.Sources.RealExpression yEva_flow_in(
    y=QEva_flow_internal/QEva_flow_nominal)
    "Normalized evaporator heat flow rate"
    annotation (Placement(transformation(extent={{28,-52},{48,-32}})));
  Modelica.Blocks.Sources.RealExpression yCon_flow_in(
    y=QCon_flow_internal/QCon_flow_nominal)
    "Normalized condenser heat flow rate"
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));

   Modelica.Blocks.Sources.RealExpression yGro_flow_in_Con(y=
        QGro_flow_internal_Con/QGro_flow_nominal_Con)
    "Normalized ground heat flow rate"
    annotation (Placement(transformation(extent={{-80,12},{-60,32}})));


  Modelica.Blocks.Math.Gain PEle_GroEva(final k=P_nominal_GroEva)
    "Electrical power consumption GroEva"
    annotation (Placement(transformation(extent={{32,80},{52,100}})));

  Modelica.Blocks.Math.Gain PEle_ConGro(final k=P_nominal_ConGro)
    "Electrical power consumption GroEva"
    annotation (Placement(transformation(extent={{30,116},{50,136}})));

   Modelica.Blocks.Sources.RealExpression yGro_flow_in_Eva(y=
        QGro_flow_internal_Eva/QGro_flow_nominal_Eva)
    "Normalized ground heat flow rate (Ground evaporator equilvent heat flow)"
    annotation (Placement(transformation(extent={{68,6},{88,26}})));
equation

  connect(PEle_GroEva.y, P_GroEva)
    annotation (Line(points={{53,90},{118,90},{118,90},{130,90}},
                                              color={0,0,127}));
  connect(PEle_GroEva.u, y_GroEva) annotation (Line(points={{30,90},{-38,90},{
          -38,-26},{-102,-26}},    color={0,0,127}));
  connect(yEva_flow_in.y, eva.u) annotation (Line(points={{49,-42},{60,-42},{60,
          -58},{20,-58}}, color={0,0,127}));
  connect(yCon_flow_in.y, con.u) annotation (Line(points={{-59,40},{-48,40},{-40,
          40},{-40,66},{-12,66}}, color={0,0,127}));
  connect(con.Q_flow, QCon_flow) annotation (Line(points={{11,66},{20,66},{80,
          66},{80,38},{113,38}},
                             color={0,0,127}));
  connect(eva.Q_flow, QEva_flow) annotation (Line(points={{-3,-58},{-20,-58},{
          -20,-40},{115,-40}},
                           color={0,0,127}));
  connect(y_ConGro, PEle_ConGro.u) annotation (Line(points={{-100,29},{-48,29},
          {-48,126},{28,126}}, color={0,0,127}));
  connect(PEle_ConGro.y, P_ConGro)
    annotation (Line(points={{51,126},{51,96},{111,96}},   color={0,0,127}));
  connect(yGro_flow_in_Con.y, groeva.u) annotation (Line(points={{-59,22},{-6,22},
          {-6,8},{-12,8}}, color={0,0,127}));
  connect(yGro_flow_in_Eva.y, congro.u) annotation (Line(points={{89,16},{96,16},
          {96,30},{56,30},{56,8},{50,8}}, color={0,0,127}));
  connect(groeva.Q_flow, QGro_flow_Con) annotation (Line(points={{-35,8},{-40,8},
          {-40,-28},{130,-28}}, color={0,0,127}));
  connect(congro.Q_flow, QGro_flow_Eva) annotation (Line(points={{27,8},{22,8},{
          22,-20},{110,-20},{110,0},{130,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}), graphics={
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
          fillColor={0,0,255},
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
        Text(
          extent={{-132,56},{-80,34}},
          textColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="y1",
          fontSize=22),
        Text(extent={{70,96},{120,82}},   textString="P",
          textColor={0,0,127}),
        Line(points={{-92,-26},{-20,-26}},            color={0,0,255}),
        Text(
          extent={{-132,-32},{-80,-54}},
          textColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          fontSize=22,
          textString="y2"),
        Line(points={{-94,30},{38,30}},               color={0,0,255}),
        Text(extent={{70,36},{120,22}},
          textColor={0,0,127},
          textString="Q"),
        Text(extent={{74,-22},{124,-36}},
          textColor={0,0,127},
          textString="Q")}),
defaultComponentName="chi",
Documentation(info="<html>
<p>
This is a partial model of a chiller whose coefficient of performance (COP) changes
with temperatures in the same way as the Carnot efficiency changes.
This base class is used for the Carnot chiller and Carnot heat pump
that uses the leaving fluid temperature as the control signal.
</p>
</html>",
revisions="<html>
<ul>
<li>
June 15, 2017, by Michael Wetter:<br/>
Added <code>min</code> attribute to parameter <code>P_nominal</code>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
Implemented in the Annex 60 library the models
<a href=\"modelica://Buildings.Fluid.Chillers.Carnot_y\">Buildings.Fluid.Chillers.Carnot_y</a>
and
<a href=\"modelica://Buildings.Fluid.HeatPumps.Carnot_y\">Buildings.Fluid.HeatPumps.Carnot_y</a>
and refactored these models to use the same base class.<br/>
Implemented the removal of the flow direction dependency of
<code>staA1</code>, <code>staB1</code>, <code>staA2</code> and <code>staB2</code> as the
efficiency of the Carnot machine should only be computed in the design flow direction,
as corrected by Damien Picard.
</li>
<li>
December 18, 2015, by Michael Wetter:<br/>
Corrected wrong computation of <code>staB1</code> and <code>staB2</code>
which mistakenly used the <code>inStream</code> operator
for the configuration without flow reversal.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/476\">
issue 476</a>.
</li>
<li>
November 25, 2015 by Michael Wetter:<br/>
Changed sign convention for <code>dTEva_nominal</code> to be consistent with
other models.
The model will still work with the old values for <code>dTEva_nominal</code>,
but it will write a warning so that users can transition their models.
<br/>
Corrected <code>assert</code> statement for the efficiency curve.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/468\">
issue 468</a>.
</li>
<li>
September 3, 2015 by Michael Wetter:<br/>
Expanded documentation.
</li>
<li>
May 6, 2015 by Michael Wetter:<br/>
Added <code>prescribedHeatFlowRate=true</code> for <code>vol2</code>.
</li>
<li>
October 9, 2013 by Michael Wetter:<br/>
Reimplemented the computation of the port states to avoid using
the conditionally removed variables <code>sta_a1</code>,
<code>sta_a2</code>, <code>sta_b1</code> and <code>sta_b2</code>.
</li>
<li>
May 10, 2013 by Michael Wetter:<br/>
Added electric power <code>P</code> as an output signal.
</li>
<li>
October 11, 2010 by Michael Wetter:<br/>
Fixed bug in energy balance.
</li>
<li>
March 3, 2009 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
end PartialCarnot_y_Z;
