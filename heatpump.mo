model HeatPump
  import ThermofluidStream.*;
  replaceable package Medium = Media.XRGMedia.R1234yf_ph constrainedby Media.myMedia.Interfaces.PartialMedium "Refrigerant Medium" annotation(
     choicesAllMatching = true);
  replaceable package Air = Media.myMedia.Air.DryAirNasa constrainedby Media.myMedia.Interfaces.PartialMedium "Air Medium" annotation(
     choicesAllMatching = true);
  HeatExchangers.DiscretizedCounterFlowHEX condenser(redeclare package MediumA = Air, redeclare package MediumB = Medium, redeclare model ConductionElementA = HeatExchangers.Internal.ConductionElementHEX, redeclare model ConductionElementB = HeatExchangers.Internal.ConductionElementHEX_twoPhase, initializeMassFlow = false, nCells = 10, A = 10, V_Hex(displayUnit = "l"), k_wall = 150) annotation(
    Placement(transformation(origin = {-37, 89}, extent = {{-17, -17}, {17, 17}}, rotation = 180)));
  HeatExchangers.DiscretizedCounterFlowHEX evaporator(redeclare package MediumA = Air, redeclare package MediumB = Medium, redeclare model ConductionElementA = HeatExchangers.Internal.ConductionElementHEX, redeclare model ConductionElementB = HeatExchangers.Internal.ConductionElementHEX_twoPhase, initializeMassFlow = false, nCells = 10, A = 10, V_Hex(displayUnit = "l") = 0.001, k_wall = 150) annotation(
    Placement(transformation(origin = {-45, -91}, extent = {{-17, -17}, {17, 17}})));
  Processes.Compressor compressor(redeclare package Medium = Medium, L = 1e6, initM_flow = ThermofluidStream.Utilities.Types.InitializationMethods.state, m_flow_0 = 0, omega_from_input = false, enableOutput = true, outputQuantity = ThermofluidStream.Sensors.Internal.Types.MassFlowQuantities.m_flow_kgps, initOmega = ThermofluidStream.Utilities.Types.InitializationMethods.state, initPhi = true, redeclare function dp_tau_compressor = Processes.Internal.TurboComponent.dp_tau_const_isentrop(omega_ref = 200, m_flow_ref = 1e-2, eta = 0.8)) annotation(
    Placement(transformation(origin = {36.5, -4.5}, extent = {{-18.5, -21.5}, {18.5, 21.5}}, rotation = 90)));
  FlowControl.BasicControlValve controlValve(redeclare package Medium = Medium, L = 5000, initM_flow = ThermofluidStream.Utilities.Types.InitializationMethods.state, invertInput = true, flowCoefficient = ThermofluidStream.FlowControl.Internal.Types.FlowCoefficientTypesBasic.Kvs, Kvs(displayUnit = "m3/s") = 0.18, m_flow_ref_set = 0.001) annotation(
    Placement(transformation(origin = {-134, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  ThermofluidStream.Examples.Utilities.Receiver receiver(redeclare package Medium = Medium, V_par(displayUnit = "l") = 0.01, p_start = 420000, init_method = ThermofluidStream.Boundaries.Internal.InitializationMethodsPhaseSeperator.h, h_0 = 220e3) annotation(
    Placement(transformation(origin = {108, -36}, extent = {{-192, 96}, {-224, 126}})));
  Boundaries.Sink sink(redeclare package Medium = Air) annotation(
    Placement(transformation(origin = {84, 12}, extent = {{-20, 80}, {0, 100}})));
  Boundaries.Source source(redeclare package Medium = Air, T0_par = 293.15, p0_par = 100000) annotation(
    Placement(transformation(origin = {2, 14}, extent = {{-160, 80}, {-140, 100}})));
  Processes.FlowResistance flowResistance1(redeclare package Medium = Air, r = 1, l = 0.5, L_value = 100, computeL = false, redeclare function pLoss = Processes.Internal.FlowResistance.laminarTurbulentPressureLoss(material = ThermofluidStream.Processes.Internal.Material.galvanizedIron)) annotation(
    Placement(transformation(origin = {-102, 104}, extent = {{-10, -10}, {10, 10}})));
  Processes.FlowResistance flowResistance2(redeclare package Medium = Air, r = 1, l = 0.5, L_value = 100, computeL = false, redeclare function pLoss = Processes.Internal.FlowResistance.laminarTurbulentPressureLoss(material = ThermofluidStream.Processes.Internal.Material.galvanizedIron)) annotation(
    Placement(transformation(origin = {2, -122}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Processes.Fan fan(redeclare package Medium = Air, initM_flow = ThermofluidStream.Utilities.Types.InitializationMethods.state, omega_from_input = true, redeclare function dp_tau_fan = Processes.Internal.TurboComponent.dp_tau_const_isentrop(omega_ref = 100)) annotation(
    Placement(transformation(origin = {82, -24}, extent = {{-84, 140}, {-56, 112}})));
  Processes.Fan fan1(redeclare package Medium = Air, initM_flow = ThermofluidStream.Utilities.Types.InitializationMethods.state, omega_from_input = true, redeclare function dp_tau_fan = Processes.Internal.TurboComponent.dp_tau_const_isentrop(omega_ref = 100)) annotation(
    Placement(transformation(origin = {-88, -115}, extent = {{-14, 19}, {14, -19}}, rotation = 180)));
  Boundaries.Source source1(redeclare package Medium = Air, temperatureFromInput = true, T0_par = 268.15, p0_par = 100000) annotation(
    Placement(transformation(origin = {42, -122}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Boundaries.Sink sink1(redeclare package Medium = Air) annotation(
    Placement(transformation(origin = {-150, -116}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.RealExpression realExpression2(y = 250) annotation(
    Placement(transformation(origin = {42, -36}, extent = {{-70, -120}, {-90, -100}})));
  Modelica.Blocks.Sources.Step vent_controll(height = -200, offset = 250, startTime = 800) annotation(
    Placement(transformation(origin = {93, -75}, extent = {{-183, 195}, {-153, 225}})));
  Sensors.TwoPhaseSensorSelect sensorVaporQuality(redeclare package Medium = Medium, quantity = ThermofluidStream.Sensors.Internal.Types.TwoPhaseQuantities.x_kgpkg) annotation(
    Placement(transformation(origin = {42, -2}, extent = {{-56, -54}, {-76, -34}})));
  Sensors.TwoPhaseSensorSelect sensorVaporQuality2(redeclare package Medium = Medium, quantity = ThermofluidStream.Sensors.Internal.Types.TwoPhaseQuantities.x_kgpkg) annotation(
    Placement(transformation(origin = {61.6, -12}, extent = {{-61.6, 34}, {-39.6, 54}})));
  Sensors.TwoPhaseSensorSelect sensorVaporQuality3(redeclare package Medium = Medium, quantity = ThermofluidStream.Sensors.Internal.Types.TwoPhaseQuantities.x_kgpkg) annotation(
    Placement(transformation(origin = {40, -10}, extent = {{-104, 34}, {-84, 54}})));
  Sensors.TwoPhaseSensorSelect sensorVaporQuality4(redeclare package Medium = Medium, quantity = ThermofluidStream.Sensors.Internal.Types.TwoPhaseQuantities.x_kgpkg) annotation(
    Placement(transformation(origin = {42, -2}, extent = {{-144, 34}, {-124, 54}})));
  Sensors.TwoPhaseSensorSelect sensorVaporQuality6(redeclare package Medium = Medium, quantity = ThermofluidStream.Sensors.Internal.Types.TwoPhaseQuantities.x_kgpkg) annotation(
    Placement(transformation(origin = {42, 2}, extent = {{-124, -54}, {-104, -34}})));
  Sensors.SingleFlowSensor singleFlowSensor(redeclare package Medium = Medium, digits = 4, quantity = ThermofluidStream.Sensors.Internal.Types.MassFlowQuantities.m_flow_kgps) annotation(
    Placement(transformation(origin = {50, -2}, extent = {{-20, 78}, {-40, 58}})));
  Sensors.MultiSensor_Tp multiSensor_Tp(redeclare package Medium = Medium, temperatureUnit = "degC", pressureUnit = "bar") annotation(
    Placement(transformation(origin = {42, 0}, extent = {{-124, -64}, {-104, -44}})));
  Sensors.MultiSensor_Tp multiSensor_Tp1(redeclare package Medium = Medium, temperatureUnit = "degC", pressureUnit = "bar") annotation(
    Placement(transformation(origin = {42, -24}, extent = {{-144, 44}, {-124, 64}})));
  Sensors.MultiSensor_Tp multiSensor_Tp2(redeclare package Medium = Medium, temperatureUnit = "degC", pressureUnit = "bar") annotation(
    Placement(transformation(origin = {40, -6}, extent = {{-104, 44}, {-84, 64}})));
  Sensors.MultiSensor_Tp multiSensor_Tp3(redeclare package Medium = Medium, temperatureUnit = "degC", pressureUnit = "bar") annotation(
    Placement(transformation(origin = {56, -6}, extent = {{-56, 44}, {-36, 64}})));
  Sensors.MultiSensor_Tp multiSensor_Tp4(redeclare package Medium = Medium, temperatureUnit = "degC", pressureUnit = "bar") annotation(
    Placement(transformation(origin = {42, -2}, extent = {{-56, -64}, {-76, -44}})));
  ThermofluidStream.Examples.Utilities.Accumulator accumulator(redeclare package Medium = Medium, useHeatport = true, U = 300, V_par(displayUnit = "l") = 0.001, p_start = 420000, init_method = ThermofluidStream.Boundaries.Internal.InitializationMethodsPhaseSeperator.l, l_0 = 0) annotation(
    Placement(transformation(origin = {44, -2}, extent = {{-40, -84}, {-20, -64}})));
  Sensors.MultiSensor_Tp multiSensor_Tp5(redeclare package Medium = Medium, temperatureUnit = "degC", pressureUnit = "bar") annotation(
    Placement(transformation(origin = {38.8, 28.8}, extent = {{7.2, -108.8}, {43.2, -74.8}})));
  Sensors.TwoPhaseSensorSelect sensorVaporQuality1(redeclare package Medium = Medium, quantity = ThermofluidStream.Sensors.Internal.Types.TwoPhaseQuantities.x_kgpkg) annotation(
    Placement(transformation(origin = {38.8, 3.4}, extent = {{7.2, -59.4}, {43.2, -37.4}})));
  Sensors.SingleSensorSelect singleSensorSelect2(redeclare package Medium = Air, digits = 1, outputValue = true, quantity = ThermofluidStream.Sensors.Internal.Types.Quantities.T_C) annotation(
    Placement(transformation(origin = {68, -66}, extent = {{-10, 100}, {10, 120}})));
  Sensors.SingleSensorSelect singleSensorSelect3(redeclare package Medium = Air, digits = 1, quantity = ThermofluidStream.Sensors.Internal.Types.Quantities.T_C) annotation(
    Placement(transformation(origin = {-6, 18}, extent = {{-120, 100}, {-100, 120}})));
  Sensors.SingleSensorSelect singleSensorSelect4(redeclare package Medium = Air, digits = 1, quantity = ThermofluidStream.Sensors.Internal.Types.Quantities.T_C) annotation(
    Placement(transformation(origin = {42, -2}, extent = {{-12, -150}, {8, -130}})));
  Sensors.SingleSensorSelect singleSensorSelect5(redeclare package Medium = Air, digits = 1, quantity = ThermofluidStream.Sensors.Internal.Types.Quantities.T_C) annotation(
    Placement(transformation(origin = {28, -30}, extent = {{-124, -128}, {-104, -108}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C = 5, T(start = 283.53), der_T(fixed = true)) annotation(
    Placement(transformation(origin = {42, -2}, extent = {{-40, -90}, {-20, -110}})));
  Sensors.TwoPhaseSensorSelect sensorVaporQuality8(redeclare package Medium = Medium, outputValue = true, quantity = ThermofluidStream.Sensors.Internal.Types.TwoPhaseQuantities.T_oversat_K, filter_output = true) annotation(
    Placement(transformation(origin = {42, -2}, extent = {{-56, -46}, {-76, -26}})));
  Modelica.Blocks.Continuous.LimPID valve_controller(controllerType = Modelica.Blocks.Types.SimpleController.PI, k = 0.1, Ti = 0.3, yMax = 1, yMin = 0, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = 1) annotation(
    Placement(transformation(origin = {118, -4}, extent = {{-162, -18}, {-198, 18}})));
  Modelica.Blocks.Sources.Constant const(k = 5) annotation(
    Placement(transformation(origin = {82, -4}, extent = {{-70, -14}, {-98, 14}})));
  Modelica.Blocks.Continuous.LimPID PI1(controllerType = Modelica.Blocks.Types.SimpleController.PI, k = 0.005, Ti = 20, yMax = 1, yMin = 0.0001, initType = Modelica.Blocks.Types.Init.InitialOutput, xi_start = 0, y_start = 0.0001) annotation(
    Placement(transformation(origin = {116, -74}, extent = {{124, 10}, {104, -10}}, rotation = 90)));
  Modelica.Blocks.Sources.Step step(height = 10, offset = 25, startTime = 500) annotation(
    Placement(transformation(origin = {116, -78}, extent = {{168, -10}, {148, 10}}, rotation = 90)));
  Modelica.Blocks.Continuous.LimPID PI2(controllerType = Modelica.Blocks.Types.SimpleController.PI, k = 10, Ti = 0.5, yMax = 10, yMin = -10, initType = Modelica.Blocks.Types.Init.InitialOutput, xi_start = 0, y_start = 0) annotation(
    Placement(transformation(origin = {42, -2}, extent = {{82, -10}, {62, 10}})));
  Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
    Placement(transformation(origin = {42, -2}, extent = {{44, -10}, {24, 10}})));
  Modelica.Blocks.Continuous.FirstOrder vent_inertion(k = 1, T = 0.1, initType = Modelica.Blocks.Types.Init.InitialState) annotation(
    Placement(transformation(origin = {57.4, -47}, extent = {{-101.4, 169}, {-75.4, 195}})));
  Modelica.Blocks.Sources.Step step2(height = -35, offset = 10 + 273.15, startTime = 300) annotation(
    Placement(transformation(origin = {42, -2}, extent = {{62, -136}, {42, -116}})));
  ThermofluidStream.Utilities.showRealValue showRealValue(use_numberPort = false, description = "CoefitientOfPerformance", number = condenser.Q_flow_A/max(0.00001, compressor.W_t)) annotation(
    Placement(transformation(origin = {-64.5405, -6.21053}, extent = {{140.541, -113.789}, {270.541, -67.7895}})));
  ThermofluidStream.Utilities.showRealValue showRealValue1(description = "Compressor Work", number = compressor.W_t, use_numberPort = false) annotation(
    Placement(transformation(origin = {-24.4324, 19.8947}, extent = {{112.432, -103.895}, {216.432, -61.8947}})));
equation
  connect(source1.outlet, flowResistance2.inlet) annotation(
    Line(points = {{32, -122}, {12, -122}}, color = {28, 108, 200}, thickness = 0.5));
  connect(flowResistance2.outlet, evaporator.inletA) annotation(
    Line(points = {{-8, -122}, {-28, -122}, {-28, -105}}, color = {28, 108, 200}, thickness = 0.5));
  connect(evaporator.outletA, fan1.inlet) annotation(
    Line(points = {{-62, -104.6}, {-61.1, -104.6}, {-61.1, -114.6}, {-74, -114.6}}, color = {28, 108, 200}, thickness = 0.5));
  connect(fan1.outlet, sink1.inlet) annotation(
    Line(points = {{-102, -115}, {-89, -115}, {-89, -116}, {-140, -116}}, color = {28, 108, 200}, thickness = 0.5));
  connect(source.outlet, flowResistance1.inlet) annotation(
    Line(points = {{-138, 104}, {-112, 104}}, color = {28, 108, 200}, thickness = 0.5));
  connect(flowResistance1.outlet, condenser.inletA) annotation(
    Line(points = {{-92, 104}, {-53, 104}, {-53, 103}, {-54, 103}}, color = {28, 108, 200}, thickness = 0.5));
  connect(condenser.outletA, fan.inlet) annotation(
    Line(points = {{-20, 102.6}, {-5, 102.6}, {-5, 101.6}, {-2, 101.6}}, color = {28, 108, 200}, thickness = 0.5));
  connect(fan.outlet, sink.inlet) annotation(
    Line(points = {{26, 102}, {64, 102}}, color = {28, 108, 200}, thickness = 0.5));
  connect(realExpression2.y, fan1.omega_input) annotation(
    Line(points = {{-49, -146}, {-63.5, -146}, {-63.5, -134}, {-88, -134}}, color = {0, 0, 127}));
  connect(condenser.outletB, receiver.inlet) annotation(
    Line(points = {{-54, 75.4}, {-84, 75.4}}, color = {28, 108, 200}, thickness = 0.5));
  connect(receiver.outlet, controlValve.inlet) annotation(
    Line(points = {{-116, 75}, {-134, 75}, {-134, 6}}, color = {28, 108, 200}, thickness = 0.5));
  connect(sensorVaporQuality3.inlet, receiver.inlet) annotation(
    Line(points = {{-64, 34}, {-68, 34}, {-68, 75}, {-84, 75}}, color = {28, 108, 200}, thickness = 0.5));
  connect(sensorVaporQuality4.inlet, controlValve.inlet) annotation(
    Line(points = {{-102, 42}, {-102, 43}, {-134, 43}, {-134, 6}}, color = {28, 108, 200}, thickness = 0.5));
  connect(multiSensor_Tp1.inlet, controlValve.inlet) annotation(
    Line(points = {{-102, 30}, {-134, 30}, {-134, 6}}, color = {28, 108, 200}, thickness = 0.5));
  connect(multiSensor_Tp2.inlet, receiver.inlet) annotation(
    Line(points = {{-64, 48}, {-68, 48}, {-68, 75}, {-84, 75}}, color = {28, 108, 200}, thickness = 0.5));
  connect(condenser.inletB, singleFlowSensor.outlet) annotation(
    Line(points = {{-20, 75.4}, {-9, 75.4}, {-9, 72.4}, {10, 72.4}}, color = {28, 108, 200}, thickness = 0.5));
  connect(evaporator.outletB, accumulator.inlet) annotation(
    Line(points = {{-28, -77.4}, {-13, -77.4}, {-13, -76.4}, {4, -76.4}}, color = {28, 108, 200}, thickness = 0.5));
  connect(multiSensor_Tp3.inlet, singleFlowSensor.outlet) annotation(
    Line(points = {{0, 48}, {-5, 48}, {-5, 72}, {10, 72}}, color = {28, 108, 200}, thickness = 0.5));
  connect(sensorVaporQuality2.inlet, singleFlowSensor.outlet) annotation(
    Line(points = {{0, 32}, {-5, 32}, {-5, 72}, {10, 72}}, color = {28, 108, 200}, thickness = 0.5));
  connect(accumulator.outlet, compressor.inlet) annotation(
    Line(points = {{24, -76}, {36.5, -76}, {36.5, -23}}, color = {28, 108, 200}, thickness = 0.5));
  connect(sensorVaporQuality.inlet, accumulator.inlet) annotation(
    Line(points = {{-14, -46}, {-8, -46}, {-8, -76}, {4, -76}}, color = {28, 108, 200}, thickness = 0.5));
  connect(multiSensor_Tp4.inlet, accumulator.inlet) annotation(
    Line(points = {{-14, -56}, {-8, -56}, {-8, -76}, {4, -76}}, color = {28, 108, 200}, thickness = 0.5));
  connect(sensorVaporQuality1.inlet, compressor.inlet) annotation(
    Line(points = {{46, -45}, {46, -23}, {36.5, -23}}, color = {28, 108, 200}, thickness = 0.5));
  connect(multiSensor_Tp5.inlet, compressor.inlet) annotation(
    Line(points = {{46, -63}, {46, -23}, {36.5, -23}}, color = {28, 108, 200}, thickness = 0.5));
  connect(singleSensorSelect2.inlet, sink.inlet) annotation(
    Line(points = {{58, 44}, {58, 68}, {64, 68}, {64, 102}}, color = {28, 108, 200}, thickness = 0.5));
  connect(singleSensorSelect3.inlet, flowResistance1.inlet) annotation(
    Line(points = {{-126, 128}, {-126, 104}, {-112, 104}}, color = {28, 108, 200}, thickness = 0.5));
  connect(singleSensorSelect4.inlet, flowResistance2.inlet) annotation(
    Line(points = {{30, -142}, {18, -142}, {18, -122}, {12, -122}}, color = {28, 108, 200}, thickness = 0.5));
  connect(singleSensorSelect5.inlet, sink1.inlet) annotation(
    Line(points = {{-96, -148}, {-111, -148}, {-111, -116}, {-140, -116}}, color = {28, 108, 200}, thickness = 0.5));
  connect(heatCapacitor.port, accumulator.heatPort) annotation(
    Line(points = {{12, -92}, {12, -84}, {14, -84}}, color = {191, 0, 0}));
  connect(sensorVaporQuality8.inlet, accumulator.inlet) annotation(
    Line(points = {{-14, -38}, {-8, -38}, {-8, -76}, {4, -76}}, color = {28, 108, 200}, thickness = 0.5));
  connect(const.y, valve_controller.u_s) annotation(
    Line(points = {{-17.4, -4}, {-40.4, -4}}, color = {0, 0, 127}));
  connect(multiSensor_Tp.inlet, evaporator.inletB) annotation(
    Line(points = {{-82, -54}, {-88, -54}, {-88, -77}, {-62, -77}}, color = {28, 108, 200}, thickness = 0.5));
  connect(sensorVaporQuality6.inlet, evaporator.inletB) annotation(
    Line(points = {{-82, -42}, {-88, -42}, {-88, -77}, {-62, -77}}, color = {28, 108, 200}, thickness = 0.5));
  connect(compressor.outlet, singleFlowSensor.inlet) annotation(
    Line(points = {{36.5, 14}, {36.5, 72}, {30, 72}}, color = {28, 108, 200}, thickness = 0.5));
  connect(controlValve.outlet, evaporator.inletB) annotation(
    Line(points = {{-134, -14}, {-134, -77}, {-62, -77}}, color = {28, 108, 200}, thickness = 0.5));
  connect(valve_controller.y, controlValve.u_in) annotation(
    Line(points = {{-81.8, -4}, {-125.8, -4}}, color = {0, 0, 127}));
  connect(torque.flange, compressor.flange) annotation(
    Line(points = {{66, -2}, {58.5, -2}, {58.5, -4.5}, {58, -4.5}}));
  connect(PI2.y, torque.tau) annotation(
    Line(points = {{103, -2}, {88, -2}}, color = {0, 0, 127}));
  connect(compressor.output_val, PI2.u_m) annotation(
    Line(points = {{58, -19.3}, {58, -28.3}, {114, -28.3}, {114, -14.3}}, color = {0, 0, 127}));
  connect(PI2.u_s, PI1.y) annotation(
    Line(points = {{126, -2}, {134.5, -2}, {134.5, 29}, {116, 29}}, color = {0, 0, 127}));
  connect(vent_controll.y, vent_inertion.u) annotation(
    Line(points = {{-58.5, 135}, {-47, 135}}, color = {0, 0, 127}));
  connect(fan.omega_input, vent_inertion.y) annotation(
    Line(points = {{12, 116}, {12, 135}, {-17, 135}}, color = {0, 0, 127}));
  connect(step2.y, source1.T0_var) annotation(
    Line(points = {{83, -128}, {64, -128}, {64, -122}, {44, -122}}, color = {0, 0, 127}));
  connect(sensorVaporQuality8.value_out, valve_controller.u_m) annotation(
    Line(points = {{-34, -38}, {-62, -38}, {-62, -26}}, color = {0, 0, 127}));
  connect(PI1.u_m, singleSensorSelect2.value_out) annotation(
    Line(points = {{104, 40}, {78, 40}, {78, 44}}, color = {0, 0, 127}));
  connect(PI1.u_s, step.y) annotation(
    Line(points = {{116, 52}, {116, 69}}, color = {0, 0, 127}));
  annotation(
    experiment(StopTime = 2500, Tolerance = 1e-6, Interval = 2.5),
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-200, -160}, {200, 160}})),
    Documentation(info = "<html>
      <p>Owner: <a href=\"mailto:michael.meissner@dlr.de\">Michael Mei&szlig;ner</a></p>
      </html>"),
    uses(ThermofluidStream(version = "1.1.0")));
end HeatPump;
