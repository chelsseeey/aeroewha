"""
System Monitor

Jetson / 일반 리눅스 환경에서 CPU / 메모리 / 디스크 / 온도 / (GPU) 사용량을
간단히 조회하고 로그로 남기기 위한 유틸.
"""

from __future__ import annotations

import logging
import platform
from typing import Any, Dict, Optional

try:
    import psutil  
except ImportError:  
    psutil = None  

logger = logging.getLogger(__name__)


class SystemMonitor:

    def __init__(self, gpu_enabled: bool = True) -> None:
        self.gpu_enabled = gpu_enabled
        self.is_jetson = "tegra" in platform.uname().release.lower()

        if psutil is None:
            logger.warning("psutil 모듈이 없어 SystemMonitor 기능이 제한됩니다.")

    # ------------------------------------------------------------------
    # 개별 항목 조회
    # ------------------------------------------------------------------
    def get_cpu_usage(self) -> Optional[float]:
        if psutil is None:
            return None

        return float(psutil.cpu_percent(interval=None))

    def get_memory_usage(self) -> Dict[str, Any]:
        if psutil is None:
            return {"percent": None, "used": None, "total": None}

        mem = psutil.virtual_memory()
        return {
            "percent": float(mem.percent),
            "used": float(mem.used),
            "total": float(mem.total),
        }

    def get_disk_usage(self, path: str = "/") -> Dict[str, Any]:
        if psutil is None:
            return {"percent": None, "used": None, "total": None, "path": path}

        disk = psutil.disk_usage(path)
        return {
            "percent": float(disk.percent),
            "used": float(disk.used),
            "total": float(disk.total),
            "path": path,
        }

    def get_temperature(self) -> Optional[float]:
        """
        시스템 온도 [°C].

        psutil.sensors_temperatures() 를 사용하고,
        사용 가능한 센서가 없으면 None 을 반환한다.
        """
        if psutil is None:
            return None

        try:
            temps = psutil.sensors_temperatures()
        except Exception:

            return None

        if not temps:
            return None

        preferred_keys = ("gpu", "nvme", "coretemp", "cpu-thermal", "soctemp")


        for key in preferred_keys:
            entries = temps.get(key)
            if entries:
                return float(entries[0].current)


        for entries in temps.values():
            if entries:
                return float(entries[0].current)

        return None

    def get_gpu_usage(self) -> Optional[Dict[str, Any]]:
        """
        GPU 사용량 정보 (가능한 경우에만).

        - NVIDIA GPU + nvidia-smi 사용 가능한 환경이면 간단히 파싱해서 리턴
        - 그렇지 않으면 None 반환
        """
        if not self.gpu_enabled:
            return None


        try:
            import subprocess

            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=utilization.gpu,memory.used,memory.total",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            line = result.stdout.strip().splitlines()[0]
            util_str, mem_used_str, mem_total_str = [v.strip() for v in line.split(",")]

            return {
                "gpu_util_percent": float(util_str),
                "mem_used_mb": float(mem_used_str),
                "mem_total_mb": float(mem_total_str),
            }
        except Exception:
            return None

    # ------------------------------------------------------------------
    # 종합 상태 + 로그
    # ------------------------------------------------------------------
    def get_status(self) -> Dict[str, Any]:
        """CPU / 메모리 / 디스크 / 온도 / GPU 상태를 하나의 dict 로 반환."""
        status: Dict[str, Any] = {
            "cpu_percent": self.get_cpu_usage(),
            "memory": self.get_memory_usage(),
            "disk": self.get_disk_usage(),
            "temperature": self.get_temperature(),
        }

        gpu_status = self.get_gpu_usage()
        status["gpu"] = gpu_status

        return status

    def log_status(self) -> None:
        
        status = self.get_status()
        cpu = status.get("cpu_percent")
        mem = status.get("memory", {}).get("percent")
        temp = status.get("temperature")


        if temp is not None:
            logger.info(
                "System Status: CPU=%.1f%%, Mem=%.1f%%, Temp=%.1f°C",
                cpu if cpu is not None else -1.0,
                mem if mem is not None else -1.0,
                temp,
            )
        else:
            logger.info(
                "System Status: CPU=%.1f%%, Mem=%.1f%%, Temp=N/A",
                cpu if cpu is not None else -1.0,
                mem if mem is not None else -1.0,
            )

        gpu = status.get("gpu")
        if gpu is not None:
            logger.info(
                "GPU Status: util=%.1f%%, mem=%.1f/%.1f MB",
                gpu.get("gpu_util_percent", 0.0),
                gpu.get("mem_used_mb", 0.0),
                gpu.get("mem_total_mb", 0.0),
            )
