<?php
class Image {

	function __construct($sqlResult) {

		$this->id = (int)$sqlResult['id'];
		$this->call = $sqlResult['call'];

		$this->time_first = (int)$sqlResult['time_first'];
		$this->time_last = (int)$sqlResult['time_last'];

		$this->imageID = (int)$sqlResult['imageID'];
		$this->packetID = (int)$sqlResult['packetID'];
		$this->count = (int)$sqlResult['count'];

	}
}
?>

