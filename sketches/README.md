# sketches

Arduino sketches for data collection and live inference.

## Arduino IDE Setup

1. Install the Arduino IDE
2. Follow the [this](https://github.com/boschsensortec/Bosch-BSEC2-Library/blob/master/README.md) guidance to install some necessary libraries and packages
3. Connect your board via USB
   - Select "Adafruit ESP32 Feather" in the Arduino IDE under Tools->Board

   - Select your COM Port under Tools->Port

4. Verify the code by clicking the check button in the top left corner
5. Make sure the baud rate is set to 921600 (or else, just make sure it matches the number in the code)
6. Upload the code to the board

NB: avoid touching the sensors while testing the device, as contamination on the surface may bias readings.

## Main Components

The primary sketches used:

1. `main.ino`: real-time inference and configuration
2. `data_collection.ino`: data collection plan execution and logging to SD card

Legacy sketches:

1. `real_time_inference.ino`: real-time inference with no baseline calculation
2. `real_time_plotting`: old version of data logging to Serial Monitor / Plotter and writing to SD card

## Data Collection

1. Define a data collection plan. Each stage has a corresponding .csv file to write data into, and is followed by a cleaning cycle.
   (a) Choose heating profile(s). The available heating profiles are defined at the top of the file, and you are always free to add more (keeping in mind hardware constraints). You can freely choose the heating profile for each sensor individually. (Note: parallel profiles often missed recording steps, possibly due to problems with sensor polling / scheduling.)
   (b) Specify other parameters of each stage as well, and flash the program.

2. Start the collection process using the `start` command, and proceed with `next` after each stage.

There are other handy commands available, e.g. `verbose` to see the logged data on the Serial Monitor.

## Deployment

To deploy a model, first use one of the notebooks to produce a `model.keras` file, and save a `train_data.csv` containing _unscaled_ features. The latter is optional, but it can be faster, as there is a handy script to make the required header file.

Convert the model into a `model.h` header file:

```
python .\scripts\model_to_tflite.py .\models\cnn\background_pca_2\model.keras -o .\models\cnn\background_pca_2\model.tflite --header .\models\cnn\background_pca_2\model.h
```

Build the `statistics.h` file from `train_data.csv`:

```
python scripts/csv_to_statistics_h.py models/cnn/background_pca_2/train_data.csv --by-position --position-col "position" --features "humidity" "humidity_diff" "gas_resistance" "gas_resistance_diff" -o models/cnn/background_pca_2/statistics.h
```

There are a number of options available in these scripts, type `--help` to view them. (Important note: statistics calculation position-wise and sensor-wise at the same time might be handy, but it's not yet implemented.) Alternatively, you can write the header file manually as well, it's quite simple, and only used to perform standard scaling.

Once you have them ready, put `model.h` and `statistics.h` into the `main` sketch folder.

In `main.ino`, pay attention to the following parts:

1. Make sure that `NUM_FEATURES`, `NUM_CLASSES`, `MODEL_TYPE`, `PREPROCESSING_TYPE` matches your setup.
2. Make sure that the right features are passed into the model and in the right order. This happens at three places in the code, always with the pattern:

```
const float raw[NUM_FEATURES] = {
        ss.fpTemp,
        ss.fpPres,
        ss.fpHum,
        ss.fpGas[t],
        ss.fpDiffTemp,
        ss.fpDiffPres,
        ss.fpDiffHum,
        ss.fpGasDiff[t],
      };
```

Once you have these in place, flash the code, check `status` with a command. Run `config <n>` to calculate baseline to correct to, if relevant. Then start inference with `run`.

You can edit the active sensors using the command `sensors <n, m, ...>`. Make sure that you run the sensors you have trained on, as there may be an offset between sensors. If you run inference with multiple sensors, the code is going to poll the sensors until a valid cycle is collected from _every_ sensor. Then these values are averaged, giving us the final input array that is passed into the network.
You might notice that this results in significant latency, since it is not rare for cycles to be dropped due to one or more missing steps. We can reduce this overhead by running `impute on`, which will replace missing values with the average across the successfully collected ones.
