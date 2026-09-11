// Native dynamics check for the generated chain/payload examples. This runs
// without ROS controllers so that the measured coupling comes from physics.
#include <mujoco/mujoco.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

int main(int argc, char** argv)
{
  try
    {
      if (argc < 3 || argc > 4)
        throw std::runtime_error("Usage: mujoco_scene_physics_test scene.xml chain|payload|elastic [plugin_library]");
      if (argc == 4) mj_loadPluginLibrary(argv[3]);
      char error[1024] = {};
      std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
      if (!model) throw std::runtime_error(error);
      std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
      auto* m = model.get();
      auto* d = data.get();
      auto body = [m](const char* name) {
        int id = mj_name2id(m, mjOBJ_BODY, name);
        if (id < 0) throw std::runtime_error(std::string("Missing body ") + name);
        return id;
      };
      if (std::string(argv[2]) == "elastic")
        {
          int a = body("elastic1/base_link"), b = body("elastic2/base_link");
          int middle = body("noodle/B_12");
          if (m->nplugin == 0 || m->body_plugin[middle] < 0)
            throw std::runtime_error("Elastic connector is not backed by a plugin");
          mj_forward(m, d);
          double initial_z = d->xpos[3 * middle + 2];
          for (int step = 0; step < 1.0 / m->opt.timestep; ++step)
            {
              d->xfrc_applied[6*a+2] = 1.3 * m->body_subtreemass[a] * 9.81;
              d->xfrc_applied[6*b+2] = 1.3 * m->body_subtreemass[b] * 9.81;
              mj_step(m, d);
            }
          mj_forward(m, d);
          for (int i = 0; i < m->nq; ++i)
            if (!std::isfinite(d->qpos[i])) throw std::runtime_error("Nonfinite cable state");
          for (int i = 0; i < mjNWARNING; ++i)
            if (d->warning[i].number) throw std::runtime_error("MuJoCo reported a dynamics warning");
          double rise = d->xpos[3 * middle + 2] - initial_z;
          double sag = (d->xpos[3*a+2] + d->xpos[3*b+2]) / 2 - d->xpos[3*middle+2];
          std::cout << "Elastic connector rise: " << rise << " m; sag: " << sag << " m\n";
          if (rise < 0.1 || sag < 0.001)
            throw std::runtime_error("Connector did not lift and bend between its endpoints");
          std::cout << "PASS\n";
          return 0;
        }
      const bool chain = std::string(argv[2]) == "chain";
      if (!chain && std::string(argv[2]) != "payload") throw std::runtime_error("Expected chain or payload");
      int a = body(chain ? "chain1/base_link" : "carrier1/main_body");
      int b = body(chain ? "chain2/base_link" : "carrier2/main_body");
      int payload = chain ? b : body("payload");
      mj_forward(m, d);
      const double initial_z = d->xpos[3 * payload + 2];
      const double separation = d->xpos[3 * b] - d->xpos[3 * a];
      const double duration = chain ? 1.0 : 1.5;
      for (int step = 0; step < duration / m->opt.timestep; ++step)
        {
          // Apply different upward loads to the two free drones. A weld must
          // preserve their offset; cables must lift the third free body.
          d->xfrc_applied[6 * a + 2] = (chain ? 1.3 : 1.15) * m->body_subtreemass[a] * 9.81;
          d->xfrc_applied[6 * b + 2] = (chain ? 1.1 : 1.15) * m->body_subtreemass[b] * 9.81;
          mj_step(m, d);
        }
      mj_forward(m, d);
      const double rise = d->xpos[3 * payload + 2] - initial_z;
      if (!std::isfinite(rise) || rise < 0.15) throw std::runtime_error("Connected body did not rise");
      std::cout << "Connected body rise: " << rise << " m\n";
      if (chain)
        {
          // Compare relative body transforms (the assembly can rotate).
          mjtNum delta[3], local[3];
          for (int i = 0; i < 3; ++i) delta[i] = d->xpos[3*b+i] - d->xpos[3*a+i];
          mju_rotVecMatT(local, delta, d->xmat + 9*a);
          double error = std::sqrt(std::pow(local[0] - separation, 2) + local[1]*local[1] + local[2]*local[2]);
          std::cout << "Weld translation error: " << error << " m\n";
          if (error > 0.01) throw std::runtime_error("Weld did not preserve robot offset");
        }
      else
        {
          if (m->ntendon != 2) throw std::runtime_error("Expected two payload cables");
          for (int i = 0; i < m->ntendon; ++i)
            {
              std::cout << "Cable " << i << " length: " << d->ten_length[i] << " m\n";
              if (d->ten_length[i] > m->tendon_range[2*i+1] + 0.01)
                throw std::runtime_error("Cable exceeded its length limit");
            }
        }
      std::cout << "PASS\n";
    }
  catch (const std::exception& ex)
    {
      std::cerr << ex.what() << '\n';
      return 1;
    }
}
