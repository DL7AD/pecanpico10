<?php
include "header.inc.php";
?>
<script type="text/javascript" src="https://www.gstatic.com/charts/loader.js"></script>
<script type="text/javascript">

lastrxtime = <?=time()-$range?>;
range = <?=$range?>;
call = '<?=$_GET['call']?>';

function drawChart() {
	updateData();
	setInterval(updateData, 1000);
}

google.charts.load('current', {'packages':['line', 'corechart']});
google.charts.setOnLoadCallback(drawChart);

</script>
<script type="text/javascript" src="script.js"></script>
<style type="text/css">
#image img {
	max-height: 110px;
}
</style>
</head>
<body>

<?php
include "sidebar.inc.php";
include "topbar.inc.php";
?>


<div class="wrapper">
	<telemetry class="inner telemetry">
		<div style="width:168px;height:42px;" title="APRS Callsign">Callsign<br><span id="call" class="fat"><?=$_GET['call']?></span></div>
		<div style="width:120px;height:42px;" title="Reset counter (incremented on each new start/reset of tracker)">Reset<br><span id="reset" class="fat"></span></div>
		<div style="width:120px;height:42px;" title="Incremental counter (imcremented on each new dataset)">ID<br><span id="id" class="fat"></span></div>
		<div style="width:250px;height:42px;" title="Time at which the packet arrived at the server">Time (RX)<br><span id="rxtime" class="fat"></span></div>
		<div style="width:174px;height:42px;" title="System time of the tracker">Time (SYS)<br><span id="sys_time" class="fat"></span></div>

		<div style="width:440px;height:135px;" title="Packet counter">
			Packets<br>
			<table>
				<tr>
					<td width="285"></td>
					<td width="100">Last Packet</td>
					<td>in 24h</td>
				</tr>
				<tr>
					<td><b>BCN</b> ______________________________</td><td><span id="act_pos"></span></td><td><span id="pos_cnt86400"></span></b></td>
				</tr>
				<tr>
					<td><b>DIR</b> ______________________________</td><td><span id="act_dir"></span></td><td><span id="dir_cnt86400"></span></b></td>
				</tr>
				<tr>
					<td><b>IMG</b> ______________________________</td><td><span id="act_img"></span></td><td><span id="img_cnt86400"></span></b></td>
				</tr>
				<tr>
					<td><b>LOG</b> ______________________________</td><td><span id="act_log"></span></td><td><span id="log_cnt86400"></span></b></td>
				</tr>
				<tr>
					<td colspan="4"><b>T-</b> &nbsp;30&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;20&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;10&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 min ago</td>
				</tr>
			</table>
		</div>
	</telemetry>


	<telemetry class="inner telemetry">

		<div style="width:440px;height:110px;background:url(power.png) no-repeat #ccdaec;" id="pwrdiv" title="Power scheme">
			<div style="position:relative;left:0px;top:0px;width:85px;">
				<span id="adc_vsol"></span>mV<sub>STM</sub><br>
				<span id="pac_vsol"></span>mV<sub>PAC</sub><br><br>
				<span id="adc_vbat"></span>mV<sub>STM</sub><br>
				<span id="pac_vbat"></span>mV<sub>PAC</sub>
			</div>
			<div style="position:relative;left:80px;top:10px;width:80px;text-align:center;"><span id="pac_psol"></span></div>
			<div style="position:relative;left:-15px;top:60px;width:80px;text-align:center;"><span id="pac_pbat"></span></div>
			<div style="position:relative;left:190px;top:35px;width:140px;"><b><span id="err_pac1720"></span></b></div>
		</div>


		<div style="width:440px;height:125px;background:url(gnss.png) no-repeat #ccdaec;" title="GNSS (GPS, GLONASS, Galileo, ...) information">
			<div style="margin-left:75px;">
				<b><span id="gps_lock" class="fat"></span></b>
				<p><span id="gps_lat" class="fat"></span> <span id="gps_lon" class="fat"></span> <img src="arrow_up.png"><span id="gps_alt" class="fat"></span></p>
				<span id="gps_sats"></span> Sats, TTFF <span id="gps_ttff"></span>s, pDOP <span id="gps_pdop"></span><br>
				Time: <span id="gps_time"></span>
			</div>
		</div>
	</telemetry>

	<telemetry class="inner telemetry">
		<div style="width:440px;height:125px;">
			<table>
				<tr>
					<td colspan="6">Sensors</td>
					<td><b>CPU</b></td>
				</tr>
				<tr>
					<td width="70"></td>
					<td width="85">Airpress.</td>
					<td width="60">Temp.</td>
					<td width="40">Hum.</td>
					<td width="60">FL</td>
					<td width="20"></td>
					<td><span id="stm32_temp"></span></td>
				</tr>
				<tr>
					<td><b>BME280<sub>I1</sub></b></td>
					<td><span id="sen_i1_press"></span></td>
					<td><span id="sen_i1_temp"></span></td>
					<td><span id="sen_i1_hum"></span></td>
					<td><span id="sen_i1_alt"></span></td>
					<td></td>
					<td><b>Transceiver</b></td>
				</tr>
				<tr>
					<td><b>BME280<sub>E1</sub></b></td>
					<td><span id="sen_e1_press"></span></td>
					<td><span id="sen_e1_temp"></span></td>
					<td><span id="sen_e1_hum"></span></td>
					<td><span id="sen_e1_alt"></span></td>
					<td></td>
					<td><span id="si4464_temp"></span></td>
				</tr>
				<tr>
					<td><b>BME280<sub>E2</sub></b></td>
					<td><span id="sen_e2_press"></span></td>
					<td><span id="sen_e2_temp"></span></td>
					<td><span id="sen_e2_hum"></span></td>
					<td><span id="sen_e2_alt"></span></td>
					<td></td>
					<td><b>Light</b></td>
				</tr>
				<tr>
					<td colspan="6"></td>
					<td><span id="light_intensity"></span><sub>Cam</sub></td>
				</tr>
			</table>
		</div>
		<div style="width:150px;height:110px;" id="image"></div>
		<div style="width:274px;height:110px;">
			<table>
				<tr>
					<td colspan="2">Subsystems</td>
				</tr>
				<tr height="5"></tr>
				<tr>
					<td width="65"><b>I<sup>2</sup>C<sub>I</sub></b></td>
					<td><b><span id="err_i2c1"></span></b></td>
				</tr>
				<tr>
					<td><b>I<sup>2</sup>C<sub>E</sub></b></td>
					<td><b><span id="err_i2c2"></span></b></td>
				</tr>
				<tr>
					<td><b>OV5640</b></td>
					<td><b><span id="err_ov5640"></span></b></td>
				</tr>
			</table>
		</div>
	</telemetry>
	<div class="inner chart">
		<div class="subheader">Power</div>
		<div id="batteryDiv"></div>
		<div id="solarDiv"></div>
	</div>
	<div class="inner chart">
		<div class="subheader">GNSS / Altitude</div>
		<div id="gpsDiv"></div>
		<div id="altDiv"></div>
	</div>
	<div class="inner chart">
		<div class="subheader">Temperature</div>
		<div id="tempDiv"></div>
	</div>
	<div class="inner chart">
		<div class="subheader">Camera</div>
		<div id="lightDiv"></div>
	</div>
	<div class="inner chart">
		<div class="subheader">Subsystem states</div>
		<div id="ioDiv"></div>
		<div id="io2Div"></div>
	</div>
	<div class="inner chart">
		<div class="subheader">Reset / ID counter</div>
		<div id="idDiv"></div>
	</div>
</div>

</body>
</html>
<?php
include "footer.inc.php";
?>

