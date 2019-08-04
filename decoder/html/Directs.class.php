<?php
class Directs {
	function __construct($sqlResult) {
		$this->rxtime = (int)$sqlResult['rxtime'];
		$this->dir = $this->getDirects($sqlResult['directs']);
	}

	function getDirects($str) {
		if($str == "[none]")
			return array();

		$data = array_fill_keys(explode(" ", $str), array());
		$query_str = "SELECT `call`,`lat`,`lon`,`rxtime` FROM `location` WHERE ";
		foreach(explode(" ", $str) as $call) {
			$query_str .= "`call` = '" . Database::getInstance()->escape_string($call) . "' OR ";
		}
		$query_str .= "0";
		$query = Database::getInstance()->query($query_str);

		while($row = $query->fetch_assoc()) {
			$data[$row['call']] = array(
				"lat" => $row['lat'],
				"lon" => $row['lon'],
				"rxtime" => $row['rxtime']
			);
		}
		return $data;
	}
}
?>

