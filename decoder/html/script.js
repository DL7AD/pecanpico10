// Common
const COL_GREEN		= "#008000";
const COL_ORANGE	= "#CC6600";
const COL_RED		= "#FF0000";

var xAxis = {
	format: range > 86400 ? 'd. H:m' : 'H:m',
	slantedTextAngle: 90
}
var area = {'width': '80%', top: 20, bottom: 70};
var scroll = {axis: 'horizontal', keepInBounds: true, maxZoomIn: 20.0};
var lastChartUpdate = 0;
var last = null;
var init = false;

// Chart 1
var batteryChart;
var solarChart;
var dataBattery;
var dataSolar;
var batteryOptions = {
	//explorer: scroll,
	height: 250,
	series: {
		0: {targetAxisIndex: 0},
		1: {targetAxisIndex: 0},
		2: {targetAxisIndex: 1}
	},
	vAxes: {
		0: {title: 'Voltage (V)', viewWindow: {min: -2000, max: 6000}, ticks: [{v: 2000, f: '2.0'}, {v: 3000, f: '3.0'}, {v: 4000, f: '4.0'}, {v: 5000, f: '5.0'}, {v: 6000, f: '6.0'}]},
		1: {title: 'Power (mW)', viewWindow: {min: -500, max: 1500}, ticks: [-500, -250, 0, 250, 500]},
	},
	legend: {
		position: 'top'
	},
	hAxis: xAxis,

	chartArea: area
};

// Chart 2
var solarOptions = {
	//explorer: scroll,
	height: 250,
	series: {
		0: {targetAxisIndex: 0},
		1: {targetAxisIndex: 0},
		2: {targetAxisIndex: 1}
	},
	vAxes: {
		0: {title: 'Voltage (V)', viewWindow: {min: -4000, max: 4000}, ticks: [{v: 0, f: '0.0'}, {v: 1000, f: '1.0'}, {v: 2000, f: '2.0'}, {v: 3000, f: '3.0'}, {v: 4000, f: '4.0'}]},
		1: {title: 'Power (mW)', viewWindow: {min: -500, max: 1500}, ticks: [-500, -250, 0, 250, 500]},
	},
	legend: {
		position: 'top'
	},
	hAxis: xAxis,

	chartArea: area
};

// Chart 3
var tempChart;
var dataTemp;
var tempOptions = {
	//explorer: scroll,
	height: 250,
	series: {
		0: {targetAxisIndex: 0},
		1: {targetAxisIndex: 0},
		2: {targetAxisIndex: 0},
		3: {targetAxisIndex: 0},
		4: {targetAxisIndex: 0}
	},
	vAxis: {
		title: 'Temp (°C)',
	},
	legend: {
		position: 'top'
	},
	hAxis: xAxis,
	chartArea: area
};

// Chart 4
var gpsChart;
var dataGPS;
var gpsOptions = {
	//explorer: scroll,
	height: 250,
	series: {
		0: {targetAxisIndex: 0},
		1: {targetAxisIndex: 1},
		2: {targetAxisIndex: 0}
	},
	vAxes: {
		0: {title: 'Sats / pDOP', viewWindow: {min: 0, max: 30}, ticks: [0,5,10,15]},
		1: {title: 'TTFF', viewWindow: {min: -180, max: 180}, ticks: [0,60,120,180]},
	},
	legend: {
		position: 'top'
	},
	hAxis: xAxis,
	chartArea: area
};

// Chart 5
var altChart;
var dataAlt;
var altOptions = {
	//explorer: scroll,
	height: 250,
	series: {
		0: {targetAxisIndex: 0},
		1: {targetAxisIndex: 1},
		2: {targetAxisIndex: 1},
		3: {targetAxisIndex: 1}
	},
	vAxes: {
		0: {title: 'Altitude'},
		1: {title: 'Airpressure'},
	},
	legend: {
		position: 'top'
	},
	hAxis: xAxis,
	chartArea: area
};

// Chart 6
var lightChart;
var dataLight;
var lightOptions = {
	//explorer: scroll,
	height: 250,
	series: {
		0: {targetAxisIndex: 0}
	},
	legend: {
		position: 'top'
	},
	hAxis: xAxis,
	chartArea: area
};

function number_format(number, decimals, decPoint, thousandsSep) { // eslint-disable-line camelcase
	//  discuss at: http://locutus.io/php/number_format/
	// original by: Jonas Raoni Soares Silva (http://www.jsfromhell.com)
	// improved by: Kevin van Zonneveld (http://kvz.io)
	// improved by: davook
	// improved by: Brett Zamir (http://brett-zamir.me)
	// improved by: Brett Zamir (http://brett-zamir.me)
	// improved by: Theriault (https://github.com/Theriault)
	// improved by: Kevin van Zonneveld (http://kvz.io)
	// bugfixed by: Michael White (http://getsprink.com)
	// bugfixed by: Benjamin Lupton
	// bugfixed by: Allan Jensen (http://www.winternet.no)
	// bugfixed by: Howard Yeend
	// bugfixed by: Diogo Resende
	// bugfixed by: Rival
	// bugfixed by: Brett Zamir (http://brett-zamir.me)
	//  revised by: Jonas Raoni Soares Silva (http://www.jsfromhell.com)
	//  revised by: Luke Smith (http://lucassmith.name)
	//    input by: Kheang Hok Chin (http://www.distantia.ca/)
	//    input by: Jay Klehr
	//    input by: Amir Habibi (http://www.residence-mixte.com/)
	//    input by: Amirouche
	number = (number + '').replace(/[^0-9+\-Ee.]/g, '');
	var n = !isFinite(+number) ? 0 : +number;
	var prec = !isFinite(+decimals) ? 0 : Math.abs(decimals);
	var sep = (typeof thousandsSep === 'undefined') ? ',' : thousandsSep;
	var dec = (typeof decPoint === 'undefined') ? '.' : decPoint;
	var s = '';
	var toFixedFix = function (n, prec) {
		var k = Math.pow(10, prec);
		return '' + (Math.round(n * k) / k).toFixed(prec);
	}
	// @todo: for IE parseFloat(0.55).toFixed(0) = 0;
	s = (prec ? toFixedFix(n, prec) : '' + Math.round(n)).split('.');
	if(s[0].length > 3) {
		s[0] = s[0].replace(/\B(?=(?:\d{3})+(?!\d))/g, sep);
	}
	if((s[1] || '').length < prec) {
		s[1] = s[1] || '';
		s[1] += new Array(prec - s[1].length + 1).join('0');
	}
	return s.join(dec);
}

function colorize(color, str) {
	return "<font color=\"" + color + "\">" + str + "</font>";
}

function time_ago(time) {
	if(time == undefined)
		return "never";

	if(time < 3600)
		return Math.floor(time/60) + "m" + (time%60) + "s ago";
	else
		return Math.floor(time/3600) + "h" + Math.floor((time/60)%60) + "m ago";
}

function time_format(time) {
	if(time == undefined)
		return "never";

	if(time < 3600)
		return Math.floor(time/60) + "m" + (time%60) + "s ago";
	else
		return Math.floor(time/3600) + "h" + Math.floor((time/60)%60) + "m ago";
}

function get_alt(p) {
	return '---'; // TODO: Implement pressure to altitude calculation
}

function updateData() {

	if(!init && typeof google !== 'undefined') {
		// Chart 1
		dataBattery = new google.visualization.DataTable();
		dataBattery.addColumn('date', 'Time');
		dataBattery.addColumn('number', "VBAT_STM");
		dataBattery.addColumn('number', "VBAT_PAC");
		dataBattery.addColumn('number', "PBAT_PAC");
		batteryChart = new google.visualization.LineChart(document.getElementById('batteryDiv'));

		// Chart 2
		dataSolar = new google.visualization.DataTable();
		dataSolar.addColumn('date', 'Time');
		dataSolar.addColumn('number', "VSOL_STM");
		dataSolar.addColumn('number', "VSOL_PAC");
		dataSolar.addColumn('number', "PSOL_PAC");
		solarChart = new google.visualization.LineChart(document.getElementById('solarDiv'));

		// Chart 3
		dataTemp = new google.visualization.DataTable();
		dataTemp.addColumn('date', 'Time');
		dataTemp.addColumn('number', "TEMP_BME_I1");
		dataTemp.addColumn('number', "TEMP_BME_E1");
		dataTemp.addColumn('number', "TEMP_BME_E2");
		dataTemp.addColumn('number', "TEMP_STM32");
		dataTemp.addColumn('number', "TEMP_Si4464");
		tempChart = new google.visualization.LineChart(document.getElementById('tempDiv'));

		// Chart 4
		dataGPS = new google.visualization.DataTable();
		dataGPS.addColumn('date', 'Time');
		dataGPS.addColumn('number', "pDOP");
		dataGPS.addColumn('number', "TTFF");
		dataGPS.addColumn('number', "Sats");
		gpsChart = new google.visualization.LineChart(document.getElementById('gpsDiv'));

		// Chart 5
		dataAlt = new google.visualization.DataTable();
		dataAlt.addColumn('date', 'Time');
		dataAlt.addColumn('number', "GPS_ALT");
		dataAlt.addColumn('number', "PRESS_BME_I1");
		dataAlt.addColumn('number', "PRESS_BME_E1");
		dataAlt.addColumn('number', "PRESS_BME_E2");
		altChart = new google.visualization.LineChart(document.getElementById('altDiv'));

		// Chart 6
		dataLight = new google.visualization.DataTable();
		dataLight.addColumn('date', 'Time');
		dataLight.addColumn('number', "LIGHT");
		lightChart = new google.visualization.LineChart(document.getElementById('lightDiv'));

		init = true;
	}

	$.getJSON("ajax/telemetry.php?call=" + call + "&from=" + lastrxtime, function(json) {

		images = json['images'];
		tel = json['telemetry'];

		// Update telemetry
		if(tel.length) {
			lastrxtime = tel[tel.length-1].rxtime+1;

			$.each(tel[tel.length-1], function(key, d) {
				switch(key) {

					case 'sen_i1_press':
						$('#sen_i1_press').text(number_format(d/10) + 'Pa');
						$('#sen_i1_alt').text(number_format(get_alt(d/10)) + 'm');
						break;

					case 'sen_e1_press':
						$('#sen_e1_press').text(number_format(d/10) + 'Pa');
						$('#sen_e1_alt').text(number_format(get_alt(d/10)) + 'm');
						break;

					case 'sen_e2_press':
						$('#sen_e2_press').text(number_format(d/10) + 'Pa');
						$('#sen_e2_alt').text(number_format(get_alt(d/10)) + 'm');
						break;

					case 'sen_i1_hum':
					case 'sen_e1_hum':
					case 'sen_e2_hum':
						$('#' + key).text(number_format(d) + '%');
						break;

					case 'sen_i1_temp':
					case 'sen_e1_temp':
					case 'sen_e2_temp':
					case 'stm32_temp':
					case 'si4464_temp':
						$('#' + key).text(number_format(d/100, 1) + '°C');
						break;

					case 'gps_pdop':
						$('#' + key).text(number_format(d/20, 2));
						break;

					case 'gps_lat':
						s  = d < 0 ? "S" : "N";
						s += Math.abs(d) < 100000000 ? "0" : "";
						s += number_format(Math.abs(d)/10000000, 4) + "°";
						$('#' + key).text(s);
						break;

					case 'gps_lon':
						s  = d < 0 ? "W" : "E";
						s += Math.abs(d) < 100000000 ? "0" : "";
						s += Math.abs(d) < 1000000000 ? "0" : "";
						s += number_format(Math.abs(d)/10000000, 4) + "°";
						$('#' + key).text(s);
						break;

					case 'gps_lock':
						switch(d) {
							case 0: $('#' + key).html(colorize(COL_GREEN, "GPS locked")); break;
							case 1: $('#' + key).html(colorize(COL_GREEN, "GPS locked - kept switched on")); break;
							case 2: $('#' + key).html(colorize(COL_RED, "GPS loss")); break;
							case 3: $('#' + key).html(colorize(COL_ORANGE, "Low Batt before switch on")); break;
							case 4: $('#' + key).html(colorize(COL_ORANGE, "Low Batt while switched on")); break;
							case 5: $('#' + key).html(colorize(COL_GREEN, "Data from memory")); break;
							case 6: $('#' + key).html(colorize(COL_RED, "GPS never locked")); break;
							case 7: $('#' + key).html(colorize(COL_RED, "GPS communication error")); break;
							case 8: $('#' + key).html(colorize(COL_ORANGE, "GPS not used - fixed loc set")); break;
						}
						break;

					case 'err_ov5640':
						switch(d) {
							case 0: $('#' + key).html(colorize(COL_GREEN, "OK")); break;
							case 1: $('#' + key).html(colorize(COL_RED, "I2C Error - Camera not found")); break;
							case 2: $('#' + key).html(colorize(COL_RED, "DMA abort - last buffer segment")); break;
							case 3: $('#' + key).html(colorize(COL_RED, "DMA FIFO error")); break;
							case 4: $('#' + key).html(colorize(COL_RED, "DMA stream transfer error")); break;
							case 5: $('#' + key).html(colorize(COL_RED, "DMA direct mode error")); break;
						}
						break;

					case 'err_pac1720':
						switch(d) {
							case 0: $('#' + key).text(''); break;
							case 1:
							case 3:
							case 2:
								$('#pac_pbat').append('<div class="cross"></div>');
								$('#pac_psol').append('<div class="cross"></div>');
								$('#' + key).html(colorize(COL_RED, "PAC1720 Fail"));
								$('#' + key).html(colorize(COL_RED, d == 2 ? "PAC1720<br>Unreliable values" : "<br>PAC1720 Fail"));
								break;
						}
						break;

					case 'err_i2c1':
					case 'err_i2c2':
					case 'err_eva7m':
						switch(d) {
							case 0: $('#' + key).html(colorize(COL_GREEN, "OK")); break;
							case 1: $('#' + key).html(colorize(COL_RED, "Fail")); break;
						}
						break;

					case 'err_bme280_i1':
					case 'err_bme280_e1':
					case 'err_bme280_e2':
						var sen = key.substr(-2, 2);
						switch(d) {
							case 1:
								$('#sen_' + sen + '_press').html(colorize(COL_RED, "Fail"));
								$('#sen_' + sen + '_temp').text('');
								$('#sen_' + sen + '_hum').text('');
								$('#sen_' + sen + '_alt').text('');
							case 2:
								$('#sen_' + sen + '_press').html(colorize(COL_ORANGE, "Not fitted"));
								$('#sen_' + sen + '_temp').text('');
								$('#sen_' + sen + '_hum').text('');
								$('#sen_' + sen + '_alt').text('');
								break;
						}
						break;

					case 'rxtime':
					case 'gps_time':
						var a = new Date(d * 1000);
						var year = a.getFullYear();
						var month = a.getMonth() < 9 ? "0" + (a.getMonth()+1) : (a.getMonth()+1);
						var date = a.getDate() < 10 ? "0" + a.getDate() : a.getDate();
						var hour = a.getHours() < 10 ? "0" + a.getHours() : a.getHours();
						var min = a.getMinutes() < 10 ? "0" + a.getMinutes() : a.getMinutes();
						var sec = a.getSeconds() < 10 ? "0" + a.getSeconds() : a.getSeconds();
						$('#' + key).text(year + '-' + month + '-' + date + ' ' + hour + ':' + min + ':' + sec);
						break;

					case 'pac_pbat':
					case 'pac_psol':
						$('#' + key).text(number_format(d/10, 1) + 'mW');
						break;

					case 'sys_time':
						$('#' + key).text(number_format(d) + 's');
						break;

					case 'gps_alt':
						$('#' + key).text(number_format(d) + 'm');
						break;

					default:
						if(Number.isInteger(d)) {
							$('#' + key).text(number_format(d));
						} else {
							$('#' + key).text(d);
						}
				}
			});
		}

		$('#pos_cnt300').text(json['packetCount']['pos']['cnt300']);
		$('#pos_cnt3600').text(json['packetCount']['pos']['cnt3600']);
		$('#pos_cnt86400').text(json['packetCount']['pos']['cnt86400']);

		$('#dir_cnt300').text(json['packetCount']['dir']['cnt300']);
		$('#dir_cnt3600').text(json['packetCount']['dir']['cnt3600']);
		$('#dir_cnt86400').text(json['packetCount']['dir']['cnt86400']);

		$('#img_cnt300').text(json['packetCount']['img']['cnt300']);
		$('#img_cnt3600').text(json['packetCount']['img']['cnt3600']);
		$('#img_cnt86400').text(json['packetCount']['img']['cnt86400']);

		$('#log_cnt300').text(json['packetCount']['log']['cnt300']);
		$('#log_cnt3600').text(json['packetCount']['log']['cnt3600']);
		$('#log_cnt86400').text(json['packetCount']['log']['cnt86400']);

		$('#act_pos').text(time_ago(json['lastActivity']['pos']));
		$('#act_img').text(time_ago(json['lastActivity']['img']));
		$('#act_log').text(time_ago(json['lastActivity']['log']));
		$('#act_dir').text(time_ago(json['lastActivity']['dir']));

		// Update charts if there is new data or at a timeout of 300 seconds
		if(tel.length > 0 || json['time']-lastChartUpdate > 300) {

			// Add new rows
			var time = json['time'];
			xAxis.minValue = new Date((time-range)*1000),
			xAxis.maxValue = new Date(time*1000),
			xAxis.ticks = [];
			interval = range / 48;
			for(i=0; i<=48; i++)
				xAxis.ticks.push(new Date(((Math.floor(time/interval)*interval)-i*interval)*1000))

			lastValidGPSalt = null;
			$.each(tel, function(key, data) {

				var time = new Date(data['org'] == 'pos' ? data['rxtime']*1000 : data['gps_time']*1000);

				if(last != null && (time - last > range*1000/60 && time - last > 300000 || time <= last)) { // Gap in the data set
					if(dataBattery) dataBattery.addRow([null, null, null, null]);
					if(dataSolar) dataSolar.addRow([null, null, null, null]);
					if(dataTemp) dataTemp.addRow([null,null,null,null,null,null]);
					if(dataGPS) dataGPS.addRow([null,null,null,null]);
					if(dataLight) dataLight.addRow([null,null]);
					if(dataAlt) dataAlt.addRow([null,null,null,null,null]);
				}

				if(dataBattery)
					dataBattery.addRow([
						time,
						data['adc_vbat'],
						data['err_pac1720'] ? null : data['pac_vbat'],
						data['pac_pbat']/10]
				);
				if(dataSolar) dataSolar.addRow([time, data['adc_vsol'], data['pac_vsol'], data['pac_psol']/10]);
				if(dataTemp) dataTemp.addRow([
					time,
					data['err_bme280_i1'] ? null : data['sen_i1_temp']/100,
					data['err_bme280_e1'] ? null : data['sen_e1_temp']/100,
					data['err_bme280_e2'] ? null : data['sen_e2_temp']/100,
					data['stm32_temp']/100,
					data['si4464_temp']/100
				]);
				if(dataGPS) dataGPS.addRow([time, data['gps_pdop']/20, data['gps_ttff'], data['gps_sats']]);
				if(dataLight) dataLight.addRow([time, data['light_intensity']]);
				if(dataAlt) {
					dataAlt.addRow([
						time,
						data['gps_lock'] < 2 ? data['gps_alt'] : lastValidGPSalt,
						data['err_bme280_i1'] ? null : data['sen_i1_press']/10,
						data['err_bme280_e1'] ? null : data['sen_e1_press']/10,
						data['err_bme280_e2'] ? null : data['sen_e2_press']/10
					]);
					if(data['gps_lock'] < 2) lastValidGPSalt = data['gps_alt'];
				}

				last = time;
			});

			// Remove old rows
			if(dataBattery) {
				var removeTime = new Date((time-range)*1000);
				for(var c=0; c<dataBattery.getNumberOfColumns(); c++) {
					if(dataBattery.getValue(c, 0) < removeTime) {
						if(dataBattery) dataBattery.removeRow(c);
						if(dataSolar) dataSolar.removeRow(c);
						if(dataTemp) dataTemp.removeRow(c);
						if(dataGPS) dataGPS.removeRow(c);
						if(dataLight) dataLight.removeRow(c);
						if(dataAlt) dataAlt.removeRow(c);
					}
				}
			}

			// Update charts
			if(batteryChart) batteryChart.draw(dataBattery, batteryOptions);
			if(solarChart) solarChart.draw(dataSolar, solarOptions);
			if(tempChart) tempChart.draw(dataTemp, tempOptions);
			if(gpsChart) gpsChart.draw(dataGPS, gpsOptions);
			if(lightChart) lightChart.draw(dataLight, lightOptions);
			if(altChart) altChart.draw(dataAlt, altOptions);

			lastChartUpdate = json['time'];
		}

		// Update images
		if(images.length) {
			lastrxtime = images[images.length-1].time_last+1;

			$.each(images, function(key, data) {
				// Remove old div
				$("#img_" + data['id']).remove();

				// Process images
				$('#images').prepend("<div class=\"pic\" id=\"img_" + data['id'] + "\">"
				 + "<img src=\"images/" + data['call'].replace('-','') + "-" + data['id'] + ".jpg?packetID=" + data['packetID'] + "\"><br>"
				 + "Last packet " + time_format(json['time']-data['time_last']) + ", " + number_format(data['count']) + " packets, "
				 + number_format(data['packetID']-data['count']+1) + " lost" + "<br>ImageID " + number_format(data['imageID']) + ", ServerID "
				 + number_format(data['id']) + "</div>");
			});

			data = images[images.length-1];
			$('#image').html("<a href=\"images.php?call=" + data['call'] + "\">"
				+ "<img src=\"images/" + images[images.length-1]['call'].replace('-','') + "-" + images[images.length-1]['id'] + ".jpg?packetID=" + data['packetID'] + "\"></a>");
		}

	});

}

