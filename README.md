# PeaceOfMine drive dashboard

## Start on the car

Run these commands on the car.

```sh
git clone https://github.com/nilskiefer/peaceofmine-drive-dashboard.git
cd peaceofmine-drive-dashboard
cd car
util/build
util/run
```

In the container shell that opens, start the operator dashboard.

```sh
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=false
```

On the device you’ll use to drive, turn on Tailscale and open [svea-mine.tailead4c5.ts.net:8080](http://svea-mine.tailead4c5.ts.net:8080/).

<details>
<summary>Develop the dashboard preview</summary>

From the repository root, run `npm install` and `npm run dev`, or use the [Lovable editor](https://lovable.dev/projects/a9d52909-a44f-4f8f-a5af-f9a6af82785b). Changes pushed to `main` sync to Lovable. [Open the hosted preview](https://peaceofmine-drive-dashboard.lovable.app).

</details>
