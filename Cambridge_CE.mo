within ;
package Cambridge_CE "The heat pump model for civil engineering building"

  model Cam_GSHP_python "Test model for heat pump based on Carnot efficiency"
    extends Modelica.Icons.Example;
    package Medium1 = Buildings.Media.Water "Medium model";
    package Medium2 =
        Buildings.Media.Antifreeze.Validation.BaseClasses.PropyleneGlycolWater
        (     property_T = 300,
              X_a = 0.05)
              "Medium model";

    package Medium3 = Buildings.Media.Water "Medium model";
    parameter Real COP_nominal_GroEva = 4.42 "Nominal COP for cooling";
    parameter Real COP_nominal_ConGro = 3.33 "Nominal COP for heaing";
    parameter Modelica.Units.SI.Power P_nominal_ConGro=72.2E3
      "Nominal compressor power for condense and ground side (at y=1)";
    parameter Modelica.Units.SI.Power P_nominal_GroEva=64E3
      "Nominal compressor power for ground and evaporator side (at y=1)";
    parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal=-6
      "Temperature difference evaporator outlet-inlet";
    parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal=5
      "Temperature difference condenser outlet-inlet";
    parameter Modelica.Units.SI.TemperatureDifference dTGro_nominal_Eva=-3
      "Temperature difference ground outlet-inlet";//10 works was condenser -10 work as evaporator
    parameter Modelica.Units.SI.TemperatureDifference dTGro_nominal_Con=5
      "Temperature difference ground outlet-inlet";//10 works was condenser -10 work as evaporator

    parameter Modelica.Units.SI.MassFlowRate m2_flow_nominal_GCon=
     (P_nominal_GroEva * (COP_nominal_GroEva+1))/cp2_default/dTGro_nominal_Con;//(condenser part )
    parameter Modelica.Units.SI.MassFlowRate m2_flow_nominal_GEva=
    -P_nominal_ConGro*(COP_nominal_ConGro-1)/cp2_default/dTGro_nominal_Eva;
        //evaporator part

    parameter Modelica.Units.SI.MassFlowRate m3_flow_nominal=-P_nominal_GroEva*
        COP_nominal_GroEva/cp3_default/dTEva_nominal
      "Nominal mass flow rate at chilled water side";  //see as evaporator

    parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal=P_nominal_ConGro*
        COP_nominal_ConGro/cp1_default/dTCon_nominal
      "Nominal mass flow rate at condenser water wide";

    Buildings.Fluid.HeatPumps.Carnot_y_Z heaPum(
      redeclare package Medium1 = Medium1,
      redeclare package Medium2 = Medium2,
      redeclare package Medium3 = Medium3,
      m2_flow_nominal_GCon= m2_flow_nominal_GCon,
      m2_flow_nominal_GEva= m2_flow_nominal_GEva,
      m1_flow_nominal= m1_flow_nominal,
      m3_flow_nominal= m3_flow_nominal,
      P_nominal_GroEva=P_nominal_GroEva,
      P_nominal_ConGro=P_nominal_ConGro,
      dTEva_nominal=dTEva_nominal,
      dTCon_nominal=dTCon_nominal,
      dTGro_nominal_Eva=dTGro_nominal_Eva,
      dTGro_nominal_Con=dTGro_nominal_Con,
      dp1_nominal=6000,
      dp2_nominal_geva=6000,
      dp2_nominal_gcon=6000,
      dp3_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      show_T=true,
      use_eta_Carnot_nominal=false,
      COP_nominal_GroEva=COP_nominal_GroEva,
      COP_nominal_ConGro=COP_nominal_ConGro,
      TCon_nominal=303.15,
      TGro_nominal=293.15,
      TEva_nominal=278.15) "Heat pump model"
      annotation (Placement(transformation(extent={{8,2},{28,22}})));
    Buildings.Fluid.Sources.MassFlowSource_T sou1(
      use_m_flow_in=true,                         nPorts=1,
      redeclare package Medium = Medium1,
      use_T_in=true,
      m_flow=m1_flow_nominal,
      T=298.15)
      annotation (Placement(transformation(extent={{-56,14},{-36,34}})));
    Buildings.Fluid.Sources.MassFlowSource_T sou2(
      use_m_flow_in=true,
      redeclare package Medium = Medium2,
      use_T_in=true,
      m_flow=max(m2_flow_nominal_GCon,m2_flow_nominal_GEva),
      T=291.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{70,-2},{50,18}})));
    Buildings.Fluid.Sources.MassFlowSource_T sou3(
      use_m_flow_in=true,                         nPorts=1,
      redeclare package Medium = Medium3,
      use_T_in=true,
      m_flow=m3_flow_nominal,
      T=295.15)
      annotation (Placement(transformation(extent={{44,-62},{24,-42}})));
    Buildings.Fluid.Sources.Boundary_pT sin1(
      redeclare package Medium = Medium1, nPorts=1)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}}, origin={124,76})));
    Buildings.Fluid.Sources.Boundary_pT Gro_Sin(redeclare package Medium =
          Medium2, nPorts=1) annotation (Placement(transformation(extent={{-5,-5},
              {5,5}}, origin={-63,-11})));
    Buildings.Fluid.Sources.Boundary_pT Bui_Coo_Sin(redeclare package Medium =
          Medium3, nPorts=1) annotation (Placement(transformation(extent={{-6,-6},
              {6,6}}, origin={-62,-46})));

    final parameter Modelica.Units.SI.SpecificHeatCapacity cp1_default=
        Medium1.specificHeatCapacityCp(Medium1.setState_pTX(
        Medium1.p_default,
        Medium1.T_default,
        Medium1.X_default))
      "Specific heat capacity of medium 2 at default medium state";

    final parameter Modelica.Units.SI.SpecificHeatCapacity cp2_default=
        Medium2.specificHeatCapacityCp(Medium2.setState_pTX(
        Medium2.p_default,
        Medium2.T_default,
        Medium2.X_default))
      "Specific heat capacity of medium 2 at default medium state";

    final parameter Modelica.Units.SI.SpecificHeatCapacity cp3_default=
        Medium3.specificHeatCapacityCp(Medium3.setState_pTX(
        Medium3.p_default,
        Medium3.T_default,
        Medium3.X_default))
      "Specific heat capacity of medium 3 at default medium state";

    Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
      tableOnFile=true,
      table=[0,14,74.5,44.9,11.3,38.4,35.3; 1,13.9,74.5,44.9,9.9,38.2,35.4; 2,14,74.5,
          44.9,11.2,38.3,35.7; 3,13.9,74.5,45,9.8,38.4,35.4; 4,14,74.5,45,11.2,38.6,
          35.6; 5,14,74.5,45,9.8,38.4,35.7; 6,14.1,74.5,45,11.2,38.3,35.5; 7,13.9,
          74.5,45.1,9.7,38.4,35.1; 8,14,74.5,45.1,11.1,38.5,35.5; 9,13.9,74.5,45.1,
          10,38.2,35.4; 10,14,74.5,45.1,11.1,38.5,35.1; 11,13.9,74.5,45.1,10.4,38.7,
          35.3; 12,14.1,74.5,45.2,11.1,38.7,35.3; 13,13.9,74.5,45.2,10.2,38.6,35.6;
          14,14.1,74.5,45.2,11,38.3,35.4; 15,13.7,74.5,45.2,11.4,38.5,35.3; 16,14.1,
          74.5,45.3,10.8,38.5,35.2; 17,13.7,74.5,45.3,11.7,38.3,35.2; 18,14.1,74.5,
          45.3,10.9,38.5,36.2; 19,14,74.5,45.3,11,38.2,35.5; 20,14.1,74.5,45.3,11,
          38.1,35.9; 21,14.1,74.5,45.4,10.8,38.7,35.6; 22,14.1,74.5,45.4,10.7,38.3,
          35.8; 23,14.2,74.5,45.4,11.6,38.9,35.7; 24,14.1,74.5,45.4,10.5,37.8,35.9;
          25,14.1,74.5,45.4,11.5,38.3,36; 26,14.1,74.5,45.5,10.4,38.2,35.7; 27,14.2,
          74.5,45.5,11.5,37.8,35.7; 28,14.1,74.5,45.5,10.2,38.3,35.4; 29,14,74.5,40.8,
          11.3,38,35.9; 30,13.1,74.5,43.1,9.9,38.3,35.8; 31,12.5,74.5,43.2,11.1,38.2,
          35.8; 32,12.5,74.5,42.9,10.2,38.1,35.5; 33,12.4,74.5,44.2,11.1,38.6,35.3;
          34,12.2,74.5,43.8,10.9,38.4,35.4; 35,12.3,74.5,43.4,10.9,38.5,35.8; 36,12.2,
          74.5,43.6,11.5,38.3,35.3; 37,12.2,74.5,43.6,10.8,38.6,35.4; 38,12.1,74.5,
          44.1,11.6,38.7,35.6; 39,12.2,74.5,44.1,10.6,38.6,35.8; 40,12,74.5,44.7,11.6,
          38.1,35.4; 41,12.2,74.5,44.8,10.4,38.2,36.1; 42,12,74.5,45.1,11.4,37.9,35.8;
          43,12.5,74.5,44.4,10.3,38.6,35.6; 44,12.1,74.5,44,11.4,38.4,35.5; 45,12.3,
          74.5,45.3,10.1,38.2,35; 46,12.3,74.5,45.3,11.3,38.7,35.4; 47,12.4,74.5,45.1,
          10,38.5,35.1; 48,12.5,74.5,44.9,11.3,38.3,35.2; 49,12.5,74.5,44.7,9.9,38.4,
          35.7; 50,12.6,74.5,44.7,11.2,38.7,36; 51,12.5,74.5,44.4,10,38.2,35.9; 52,
          12.6,74.5,43.9,11.2,38.2,35.7; 53,12.7,74.5,43.8,9.8,38.3,35.4; 54,12.6,
          74.5,43.7,11.2,38.2,35.6; 55,12.8,74.5,43.6,9.7,38.3,35.8; 56,12.9,74.5,
          43.8,11.1,38.1,35.6; 57,12.7,74.5,44,10.3,38,35.3; 58,13,74.5,44.4,11,38.6,
          35.6; 59,12.7,74.5,44.7,10.7,38.7,35.6; 60,13,74.5,45.4,10.9,38.3,35.4;
          61,13,74.5,44.7,11.2,38.4,35.8; 62,13.1,74.5,43.8,10.9,38.5,35; 63,12.9,
          74.5,44.1,11.7,38.1,35.7; 64,13.3,74.5,44.6,10.8,38.4,35.7; 65,12.9,74.5,
          45.1,11.7,38.6,35.6; 66,13.2,74.5,45.5,10.7,37.9,35.5; 67,13.1,74.5,45.3,
          11.6,38.4,34.9; 68,13.4,74.5,44.1,10.5,38.4,36.4; 69,13.2,74.5,44.1,11.5,
          38.4,35.7; 70,13.3,74.5,45.3,10.4,38.5,35.5; 71,13.5,74.5,44,11.5,37.9,35.7;
          72,13.4,74.5,45.4,10.4,38.5,35.7; 73,13.3,74.5,43.8,11.5,38.3,35.7; 74,13.3,
          74.5,44.4,10.2,37.8,35.4; 75,13.3,74.5,45.5,11.4,38.6,35.9; 76,13.5,74.5,
          43.8,10.1,38.2,35.5; 77,13.4,74.5,44.4,11.3,38.3,35.4; 78,13.3,74.5,45.4,
          9.8,38.3,35.3; 79,13.4,74.5,45.5,11.1,38.3,35.4; 80,13.2,74.5,44.3,10.2,
          38.2,35.5; 81,13.3,74.5,43.7,11.1,38.3,35.8; 82,13.1,74.5,43.8,10.3,38.3,
          35.8; 83,13.4,74.5,44.1,10.9,38.3,35.4; 84,13.1,74.5,44.8,11.7,38.5,36;
          85,13.4,74.5,46.1,10.6,37.7,35.5; 86,13.5,74.5,46,11.6,38.8,35.2; 87,13.5,
          74.5,46,10.4,38.8,35.7; 88,13.6,74.5,46,11.5,38.5,35.9; 89,13.7,74.5,46,
          10.3,38.2,35.4; 90,13.7,74.5,46,11.6,38.3,35.1; 91,13.8,74.5,46,10.6,38.6,
          35.6; 92,13.9,74.5,46,11.7,38.4,35.4; 93,13.8,74.5,46,10.5,38.2,35.8; 94,
          13.9,74.5,46,11.5,38.6,35.3; 95,13.8,74.5,46.1,10.3,38.3,35.9; 96,13.9,74.5,
          46,11.4,38.2,36.1],
      tableName="table",
      fileName=ModelicaServices.ExternalReferences.loadResource("modelica://Buildings/GSHP_input_data.txt"),
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
      timeScale=900)
      annotation (Placement(transformation(extent={{-260,-80},{-240,-60}})));

    Modelica.Fluid.Sensors.TemperatureTwoPort Gro_Ent_T(redeclare package
        Medium =
          Medium2)
      annotation (Placement(transformation(extent={{-30,-10},{-40,0}})));
    Modelica.Fluid.Sensors.TemperatureTwoPort GSHP_to_Hot_Buffer_T(redeclare
        package Medium = Medium1)
      annotation (Placement(transformation(extent={{32,68},{52,90}})));
    Modelica.Fluid.Sensors.TemperatureTwoPort GSHP_to_Cool_Buffer_T(redeclare
        package Medium = Medium3)
      annotation (Placement(transformation(extent={{-24,-52},{-36,-40}})));
    Modelica.Blocks.Logical.Hysteresis hys(uLow=273.15 + 44, uHigh=273.15 + 46)
      "Hysteresis controller"
      annotation (Placement(transformation(extent={{-200,86},{-180,106}})));
    Modelica.Blocks.Math.BooleanToReal booHea(realTrue=0, realFalse=1)
      "Conversion to real control signal"
      annotation (Placement(transformation(extent={{-160,86},{-140,106}})));
    Modelica.Blocks.Logical.Hysteresis hys1(uLow=273.15 + 9, uHigh=273.15 + 12)
      "Hysteresis controller"
      annotation (Placement(transformation(extent={{-196,44},{-176,64}})));
    Modelica.Blocks.Math.BooleanToReal booCoo(realTrue=1, realFalse=0)
      "Conversion to real control signal"
      annotation (Placement(transformation(extent={{-158,44},{-138,64}})));
    parameter Modelica.Units.SI.Time samplePeriod=0.01
      "Sample period of component";

    Buildings.Utilities.IO.Python_3_8.Real_Real addcon(
      nDblWri=3,
      nDblRea=2,
      functionName="constant",
      moduleName="GroundHX",
      samplePeriod=800,
      passPythonObject=true) "add constant degree after ground heat exchanger"
      annotation (Placement(transformation(extent={{28,-108},{48,-88}})));
    Buildings.Utilities.Time.ModelTime modTim "Model time"
      annotation (Placement(transformation(extent={{-132,-108},{-112,-88}})));
    Modelica.Blocks.Routing.Multiplex3 multiplex3_1
      annotation (Placement(transformation(extent={{-66,-108},{-46,-88}})));
    Modelica.Fluid.Sensors.MassFlowRate massFlowRate(redeclare package Medium
        = Medium2) "senor for flowrate"
      annotation (Placement(transformation(extent={{-26,-30},{-38,-20}})));
  equation
    connect(sou1.ports[1], heaPum.port_a1) annotation (Line(
        points={{-36,24},{-10,24},{-10,15.3333},{9.42857,15.3333}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(sou3.ports[1], heaPum.port_a3)
      annotation (Line(points={{24,-52},{23.7143,-52},{23.7143,5.16667}},
                                                            color={0,127,255}));
    connect(combiTimeTable.y[4], sou3.T_in) annotation (Line(points={{-239,-70},{
            52,-70},{52,-48},{46,-48}},                                   color={0,
            0,127}));
    connect(combiTimeTable.y[3], sou1.T_in) annotation (Line(points={{-239,-70},{
            -204,-70},{-204,28},{-58,28}},             color={0,0,127}));
    connect(heaPum.port_b2, Gro_Ent_T.port_a) annotation (Line(points={{9.57143,
            10.6667},{-24,10.6667},{-24,-5},{-30,-5}}, color={0,127,255}));
    connect(Gro_Ent_T.port_b, Gro_Sin.ports[1]) annotation (Line(points={{-40,-5},
            {-48,-5},{-48,-11},{-58,-11}}, color={0,127,255}));
    connect(heaPum.port_b3, GSHP_to_Cool_Buffer_T.port_a) annotation (Line(points={{9.71429,
            4.83333},{9.71429,10},{-14,10},{-14,-46},{-24,-46}},
          color={0,127,255}));
    connect(GSHP_to_Cool_Buffer_T.port_b, Bui_Coo_Sin.ports[1])
      annotation (Line(points={{-36,-46},{-56,-46}}, color={0,127,255}));
    connect(heaPum.port_b1, GSHP_to_Hot_Buffer_T.port_a) annotation (Line(points={{23.7143,
            15.3333},{23.7143,18},{34,18},{34,60},{20,60},{20,79},{32,79}},
          color={0,127,255}));
    connect(GSHP_to_Hot_Buffer_T.port_b, sin1.ports[1]) annotation (Line(points={{52,79},
            {52,76},{114,76}},                       color={0,127,255}));
    connect(combiTimeTable.y[6], sou3.m_flow_in) annotation (Line(points={{-239,
            -70},{52,-70},{52,-44},{46,-44}},
          color={0,0,127}));
    connect(combiTimeTable.y[5], sou1.m_flow_in) annotation (Line(points={{-239,
            -70},{-204,-70},{-204,28},{-64,28},{-64,32},{-58,32}},
                                          color={0,0,127}));
    connect(booHea.y, heaPum.y_ConGro) annotation (Line(points={{-139,96},{2,96},
            {2,12.75},{9.42857,12.75}}, color={0,0,127}));
    connect(hys.y, booHea.u)
      annotation (Line(points={{-179,96},{-162,96}}, color={255,0,255}));
    connect(combiTimeTable.y[3], hys.u) annotation (Line(points={{-239,-70},{-204,
            -70},{-204,80},{-210,80},{-210,96},{-202,96}}, color={0,0,127}));
    connect(booCoo.y, heaPum.y_GroEva) annotation (Line(points={{-137,54},{-24,
            54},{-24,16},{0,16},{0,8.16667},{9.28571,8.16667}}, color={0,0,127}));
    connect(combiTimeTable.y[4], hys1.u) annotation (Line(points={{-239,-70},{
            -228,-70},{-228,54},{-198,54}},                color={0,0,127}));
    connect(hys1.y, booCoo.u)
      annotation (Line(points={{-175,54},{-160,54}}, color={255,0,255}));
    connect(modTim.y, multiplex3_1.u3[1]) annotation (Line(points={{-111,-98},{
            -108,-98},{-108,-105},{-68,-105}}, color={0,0,127}));
    connect(Gro_Ent_T.T, multiplex3_1.u1[1]) annotation (Line(points={{-35,0.5},
            {-35,-2},{-98,-2},{-98,-91},{-68,-91}},
                                                 color={0,0,127}));
    connect(multiplex3_1.y, addcon.uR)
      annotation (Line(points={{-45,-98},{26,-98}}, color={0,0,127}));
    connect(addcon.yR[1], sou2.T_in) annotation (Line(points={{49,-98.25},{86,
            -98.25},{86,12},{72,12}}, color={0,0,127}));
    connect(heaPum.port_b2, massFlowRate.port_a) annotation (Line(points={{
            9.57143,10.6667},{-24,10.6667},{-24,-16},{-18,-16},{-18,-25},{-26,
            -25}}, color={0,127,255}));
    connect(massFlowRate.m_flow, multiplex3_1.u2[1]) annotation (Line(points={{
            -32,-19.5},{-32,-22},{-82,-22},{-82,-98},{-68,-98}}, color={0,0,127}));
    connect(sou2.ports[1], heaPum.port_a2) annotation (Line(points={{50,8},{34,
            8},{34,10.3333},{23.7143,10.3333}}, color={0,127,255}));
    connect(combiTimeTable.y[2], sou2.m_flow_in) annotation (Line(points={{-239,
            -70},{-78,-70},{-78,-114},{110,-114},{110,16},{72,16}}, color={0,0,
            127}));
    annotation (experiment(
        StopTime=86400,
        Interval=900,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
  __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/HeatPumps/Examples/Carnot_y.mos"
          "Simulate and plot"),
      Documentation(revisions="<html>
<ul>
<li>
May 15, 2019, by Jianjun Hu:<br/>
Replaced fluid source. This is for 
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1072\"> #1072</a>.
</li>
<li>
November 25, 2015 by Michael Wetter:<br/>
Changed sign of <code>dTEva_nominal</code> to be consistent.
</li>
<li>
December 22, 2014 by Michael Wetter:<br/>
Removed <code>Modelica.Fluid.System</code>
to address issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/311\">#311</a>.
</li>
<li>
March 26, 2013 by Michael Wetter:<br/>
Removed assignment of parameter that had attribute <code>fixed=false</code>.
</li>
<li>
March 3, 2009 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>",   info="<html>
<p>
Example that simulates a heat pump whose efficiency is scaled based on the
Carnot cycle.
The control signal of the heat pump is the compressor speed.
</p>
</html>"),
      Diagram(coordinateSystem(extent={{-280,-120},{160,120}}), graphics={
          Text(
            extent={{48,2},{72,-12}},
            textColor={28,108,200},
            textString="Water temperature from ground side
"),       Text(
            extent={{68,-66},{-8,-62}},
            textColor={28,108,200},
            textString="Water temperature from cool tank"),
          Text(
            extent={{-66,-24},{-58,-26}},
            textColor={28,108,200},
            textString="
"),       Text(
            extent={{-64,24},{-138,20}},
            textColor={28,108,200},
            textString="Water temperature from hot tank"),
          Text(
            extent={{-160,114},{-234,110}},
            textColor={28,108,200},
            textString="Water temperature from hot tank"),
          Text(
            extent={{-154,36},{-230,40}},
            textColor={28,108,200},
            textString="Water temperature from cool tank"),
          Text(
            extent={{96,94},{20,98}},
            textColor={28,108,200},
            textString="Building heating zone sink
")}), Icon(coordinateSystem(extent={{-280,-120},{160,120}})));
  end Cam_GSHP_python;
  annotation (uses(
      Modelica(version="4.0.0"),
      Buildings(version="9.1.0"),
      ModelicaServices(version="4.0.0")));
end Cambridge_CE;
