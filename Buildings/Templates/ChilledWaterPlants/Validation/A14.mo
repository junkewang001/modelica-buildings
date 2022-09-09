within Buildings.Templates.ChilledWaterPlants.Validation;
model A14
  "Parallel chillers, variable primary, constant speed CW pumps, headered pumps"
  extends Buildings.Templates.ChilledWaterPlants.Validation.BaseWaterCooled(
      redeclare
      Buildings.Templates.ChilledWaterPlants.Validation.UserProject.A14 chw);
  annotation (experiment(Tolerance=1e-6, StopTime=1));
end A14;
