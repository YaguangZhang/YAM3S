# YAM3S

The code base for Yet Another Mobile Millimeter-wave Measurement System (YAM3S). YAM3S combines real-time kinematic (RTK) GPS, inertial measurement units (IMUs) with 2-axis rotators to automatically align the transmitter (TX) and receiver (RX) antennas. It is designed for a mobile RX with the TX installed at a fixed location.

## Checklist

Assuming that `settings.json` is filled out correctly, then for each measurement
recording, the steps one needs to carry out are listed below.

1. Start the remote mySQL server.
2. Manually adjust the TX and RX servos so that the antennas are aligned with
   enough moving/rotating room for the rotators to cover the area of interest.
3. Measure or find on a map the TX and RX GPS locations and update the
   `settings.json` file accordingly.
4. If desired, assign a string label to the measurement recording attempt to
   `measurement_attempt_label` in `settings.json`.
5. On both the TX and the RX sides, start (i) arduino, (ii) the Python
   controller on the connect computer.

## Contact

* **Yaguang Zhang** | *Purdue University* | Email: ygzhang@purdue.edu

## License

This project is licensed under the MIT License.
