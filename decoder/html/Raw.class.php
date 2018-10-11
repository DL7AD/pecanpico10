<?php
class Raw {
	function __construct($sqlResult) {
		$this->rxtime = (int)$sqlResult['rxtime'];
		$this->call = $sqlResult['call'];
		$this->data = $sqlResult['data'];
		$this->meta = json_decode($sqlResult['meta']);
	}
}
?>

