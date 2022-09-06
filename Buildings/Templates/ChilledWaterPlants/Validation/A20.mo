within Buildings.Templates.ChilledWaterPlants.Validation;
model A20 "Parallel chillers, primary-secondary, constant speed CW
pumps, headered pumps"
  extends Buildings.Templates.ChilledWaterPlants.Validation.BaseWaterCooled(
      redeclare
      Buildings.Templates.ChilledWaterPlants.Validation.UserProject.RP1711_6_7
      chw);
  annotation (experiment(Tolerance=1e-6, StopTime=1));
end A20;
