<?php
class Telemetry {
	function __construct($sqlResult) {
		$this->reset = (int)$sqlResult['reset'];
		$this->id = (int)$sqlResult['id'];
		$this->org = $sqlResult['org'];

		$this->rxtime = (int)$sqlResult['rxtime'];

		$this->call = $sqlResult['call'];

		$this->adc_vsol = (int)$sqlResult['adc_vsol'];
		$this->adc_vbat = (int)$sqlResult['adc_vbat'];
		$this->pac_vsol = (int)$sqlResult['pac_vsol'];
		$this->pac_vbat = (int)$sqlResult['pac_vbat'];
		$this->pac_pbat = (int)$sqlResult['pac_pbat'];
		$this->pac_psol = (int)$sqlResult['pac_psol'];

		$this->gps_time = (int)$sqlResult['gps_time'];
		$this->gps_lock = (int)$sqlResult['gps_lock'];
		$this->gps_sats = (int)$sqlResult['gps_sats'];
		$this->gps_ttff = (int)$sqlResult['gps_ttff'];
		$this->gps_pdop = (int)$sqlResult['gps_pdop'];
		$this->gps_alt = (int)$sqlResult['gps_alt'];
		$this->gps_lat = (int)$sqlResult['gps_lat'];
		$this->gps_lon = (int)$sqlResult['gps_lon'];

		$this->sen_i1_press = (int)$sqlResult['sen_i1_press'];
		$this->sen_e1_press = (int)$sqlResult['sen_e1_press'];
		$this->sen_e2_press = (int)$sqlResult['sen_e2_press'];
		$this->sen_i1_temp = (int)$sqlResult['sen_i1_temp'];
		$this->sen_e1_temp = (int)$sqlResult['sen_e1_temp'];
		$this->sen_e2_temp = (int)$sqlResult['sen_e2_temp'];
		$this->sen_i1_hum = (int)$sqlResult['sen_i1_hum'];
		$this->sen_e1_hum = (int)$sqlResult['sen_e1_hum'];
		$this->sen_e2_hum = (int)$sqlResult['sen_e2_hum'];

		$this->stm32_temp = (int)$sqlResult['stm32_temp'];
		$this->si4464_temp = (int)$sqlResult['si4464_temp'];

		$this->light_intensity = (int)$sqlResult['light_intensity'];

		$this->sys_time = (int)$sqlResult['sys_time'];
		$this->sys_error = (int)$sqlResult['sys_error'];

		$this->err_i2c1 = ((int)$this->sys_error >> 0) & 0x1;
		$this->err_i2c2 = ((int)$this->sys_error >> 1) & 0x1;
		$this->err_eva7m = ((int)$this->sys_error >> 2) & 0x1;
		$this->err_pac1720 = ((int)$this->sys_error >> 3) & 0x3;
		$this->err_ov5640 = ((int)$this->sys_error >> 5) & 0x3;
		$this->err_bme280_i1 = ((int)$this->sys_error >> 8) & 0x1;
		$this->err_bme280_e1 = ((int)$this->sys_error >> 9) & 0x1;
		$this->err_bme280_e2 = ((int)$this->sys_error >> 10) & 0x1;

	}
}
?>

