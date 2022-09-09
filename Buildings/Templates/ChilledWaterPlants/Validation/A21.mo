within Buildings.Templates.ChilledWaterPlants.Validation;
model A21 "Parallel chillers with WSE, primary-secondary, variable speed
CW pumps, headered pumps"
  extends Buildings.Templates.ChilledWaterPlants.Validation.BaseWaterCooled(
      redeclare
      Buildings.Templates.ChilledWaterPlants.Validation.UserProject.A21 chw);
  annotation (experiment(Tolerance=1e-6, StopTime=1));
end A21;
